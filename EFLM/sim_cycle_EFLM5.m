function r = sim_cycle_EFLM5(a,p,tab,cfg,n)
% Five-coordinate, massless-leg Lagrange system from urls.
% Translational equations integrated analytically; angular algebraic equations
% solved directly, without repeatedly invoking symbolic DAE index reduction.
watch=tic; b=prescribed_motion(a,p,n); Y=-[b.x,b.z]; G=[b.Fx,b.Fz];
rad=sqrt(sum(Y.^2,2));
k=ppval(tab.inverse,min(tab.lRange(2),max(tab.lRange(1),rad)));
qa=ppval(tab.ankle,k); d=eflm5_geometry([zeros(n,1),k,qa],p);
h=atan2(-Y(:,2),Y(:,1))-atan2(-d.toe(:,2),d.toe(:,1));
angles=[h,k,qa]; scale=[p.lengthScale,p.lengthScale,p.tauMax]; converged=false;
for it=1:60
 [e,d]=eflm5_balance(angles,Y,G,p);
 if any(~isfinite(e),'all'), error('EFLM:Geometry','Nonfinite five-coordinate balance'); end
 if max(abs(e(:,1:2)),[],'all')<1e-12 && max(abs(e(:,3)))<1e-10
  converged=true; break;
 end
 J=eflm5_jacobian(angles,Y,G,p);
 if any(abs(pagedet3(J))<1e-10), error('EFLM:Singular','Five-coordinate Jacobian singular'); end
 step=reshape(pagemldivide(J,reshape(e',3,1,n)),3,n)';
 factor=1./max(1,max(abs(step),[],2)/.12); merit=sum((e./scale).^2,2);
 for bt=1:24
  trial=angles-factor.*step; [et,dt]=eflm5_balance(trial,Y,G,p);
  bad=any(~isfinite(et),2) | trial(:,2)<tab.kRange(1) | trial(:,2)>tab.kRange(2)...
   | (sum((et./scale).^2,2)>merit & merit>1e-23);
  % Keep the original closed-chain assembly, not the crossed branch.
  [~,branch]=eflm_geometry(trial(:,1),trial(:,2),dt.s,p);
  bad=bad | abs(imag(branch))>1e-12 | abs(real(branch)-trial(:,3))>1e-6;
  if ~any(bad), break; end
  factor(bad)=factor(bad)/2;
 end
 if any(bad), error('EFLM:Convergence','Five-coordinate Newton line search failed'); end
 angles=trial;
 if toc(watch)>cfg.evaluationSeconds, error('EFLM:Timeout','Five-coordinate evaluation timeout'); end
end
if ~converged, error('EFLM:Convergence','Five-coordinate equilibrium did not converge'); end
J=eflm5_jacobian(angles,Y,G,p);
rhs=[-b.vx,-b.vz,d.Jx(:,3).*b.dFx+d.Jz(:,3).*b.dFz];
rates=reshape(pagemldivide(J,reshape(rhs',3,1,n)),3,n)';
s=d.s; sd=sum(d.ds.*rates,2); tau=d.requiredTorque(:,1:2); dq=rates(:,1:2);
% Export full generalized-coordinate equations and their power balance.
q5=[b.x,b.z,angles]; q5d=[b.vx,b.vz,rates];
residual5=[zeros(n,2),d.requiredTorque-[tau,zeros(n,1)]];
[lo,hi,rpm]=motor_envelope(dq,p);
power=tau.*dq; energy=trapz(b.t,sum(abs(power),2)); cot=energy/(p.m*p.g*b.distance);
channels=[(tau-hi)/p.tauMax,(lo-tau)/p.tauMax,...
 angles(:,1:2)-p.jointUpper,p.jointLower-angles(:,1:2),...
 (s-p.smax)/p.smax,-s/p.smax,(abs(rpm)-p.rpmLimit)/p.rpmLimit,-b.z/p.lengthScale];
durationConstraints=[(cfg.minimumStance-b.Ts)/cfg.minimumStance;...
 (cfg.minimumFlight-b.Tf)/cfg.minimumFlight];
c=[channels(:);-b.distance;durationConstraints];
[springForce,springStiffness,springEnergy]=spring_response(s,p,'smoothDiagnostic');
% Legacy plotting coordinates are reference (ds=0) leg length/orientation.
k=angles(:,2); h=angles(:,1); epsc=1e-20;
[ref,~]=eflm_geometry(h,k,zeros(n,1),p);
refdot=zeros(n,2);
for j=1:2
 hc=h; kc=k; if j==1, hc=hc+1i*epsc; else, kc=kc+1i*epsc; end
 refdot=refdot+imag(eflm_geometry(hc,kc,zeros(n,1),p))/epsc.*rates(:,j);
end
l=sqrt(sum(ref.^2,2)); ld=sum(ref.*refdot,2)./l;
q=atan2(-ref(:,2),ref(:,1)); qd=(ref(:,2).*refdot(:,1)-ref(:,1).*refdot(:,2))./l.^2;
Tr=Y(:,1).*G(:,2)-Y(:,2).*G(:,1); Fr=-sum(Y.*G,2)./rad;
r=struct('a',a,'body',b,'s',s,'sd',sd,'l',l,'ld',ld,'q',q,'qd',qd,...
 'angles',angles,'angularVelocity',rates,'tau',tau,'rpm',rpm,...
 'torqueLower',lo,'torqueUpper',hi,'power',power,'energy',energy,'cot',cot,...
 'objective',cfg.w2*cot-cfg.w1*b.speed,'constraints',c,'maxViolation',max([0;c]),...
 'constraintChannels',channels,'constraintEnvelope',[max(channels,[],1)';-b.distance;durationConstraints],...
 'durationConstraints',durationConstraints,'equilibriumResidual',[sqrt(sum(e(:,1:2).^2,2)),e(:,3)],...
 'pointsLocal',d.points,'jacobian',J,'Fr',Fr,'tauR',Tr,...
 'springForce',springForce,'springStiffness',springStiffness,'springEnergy',springEnergy,...
 'linearSpringEnergy',.5*p.ks*s.^2,'additionalSpringTorque',d.extraTorque,...
 'additionalSpringPower',sum(d.extraTorque.*rates,2),...
 'generalizedCoordinates',q5,'generalizedVelocity',q5d,'lagrangeResidual',residual5,...
 'contactResidual',e(:,1:2),'cumulativeAbsoluteWork',cumtrapz(b.t,abs(power)),...
 'exactToeResidual',d.toe-Y,'iterations',it,'elapsed',toc(watch),'success',true);
end

function detj=pagedet3(A)
detj=squeeze(A(1,1,:).*(A(2,2,:).*A(3,3,:)-A(2,3,:).*A(3,2,:))...
 -A(1,2,:).*(A(2,1,:).*A(3,3,:)-A(2,3,:).*A(3,1,:))...
 +A(1,3,:).*(A(2,1,:).*A(3,2,:)-A(2,2,:).*A(3,1,:)));
end
