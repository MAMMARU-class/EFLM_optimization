function r = sim_cycle_elastic_approx(a,p,tab,cfg,n)
% Reduced DAE: analytically integrated translation plus coupled algebraic solve.
watch=tic; b=prescribed_motion(a,p,n); Y=[-b.x,-b.z]; V=[-b.vx,-b.vz];
G=-[b.Fx,b.Fz]; Gd=-[b.dFx,b.dFz];
rad=sqrt(sum(Y.^2,2)); rd=sum(Y.*V,2)./rad;
theta=atan2(-Y(:,2),Y(:,1)); td=(Y(:,2).*V(:,1)-Y(:,1).*V(:,2))./rad.^2;
Fr=sum(Y.*G,2)./rad; Tr=Y(:,2).*G(:,1)-Y(:,1).*G(:,2);
Frd=(sum(V.*G+Y.*Gd,2)-Fr.*rd)./rad;
Trd=V(:,2).*G(:,1)+Y(:,2).*Gd(:,1)-V(:,1).*G(:,2)-Y(:,1).*Gd(:,2);
k=ppval(tab.inverse,min(tab.lRange(2),max(tab.lRange(1),rad)));
s=zeros(n,1); converged=false;
for it=1:60
 l=ppval(tab.l,k); ld=ppval(tab.ld,k);
 f=ppval(tab.f,k); fd=ppval(tab.fd,k); g=ppval(tab.g,k); gd=ppval(tab.gd,k);
 [Fs,K]=spring_response(s,p);
 e1=l-f.*s-rad; e2=Fs-Fr.*f-Tr.*g;
 if max(abs(e1))<1e-13 && max(abs(e2))<1e-10, converged=true; break; end
 aa=ld-fd.*s; bb=-f; cc=-Fr.*fd-Tr.*gd; dd=K;
 detj=aa.*dd-bb.*cc;
 if any(abs(detj)<1e-6), error('EFLM:Singular','Algebraic Jacobian singular'); end
 dk=(dd.*e1-bb.*e2)./detj; ds=(-cc.*e1+aa.*e2)./detj;
 scale=max(1,max(abs(dk)/.10,abs(ds)/.003));
 merit=(e1/p.leg).^2+(e2/(p.ks*p.smax)).^2;
 factor=1./scale;
 for backtrack=1:18
  kt=k-factor.*dk; st=s-factor.*ds;
  et1=ppval(tab.l,kt)-ppval(tab.f,kt).*st-rad;
  et2=spring_response(st,p)-Fr.*ppval(tab.f,kt)-Tr.*ppval(tab.g,kt);
  trial=(et1/p.leg).^2+(et2/(p.ks*p.smax)).^2;
  bad=trial>merit & merit>1e-22;
  if ~any(bad), break; end
  factor(bad)=factor(bad)/2;
 end
 k=kt; s=st;
 if any(k<tab.kRange(1)|k>tab.kRange(2)), error('EFLM:Geometry','Outside geometry branch'); end
 if toc(watch)>cfg.evaluationSeconds, error('EFLM:Timeout','Evaluation exceeded time budget'); end
end
if ~converged, error('EFLM:Convergence','Algebraic equilibrium did not converge'); end
aa=ld-fd.*s; bb=-f; cc=-Fr.*fd-Tr.*gd; detj=aa.*K-bb.*cc;
rhs2=Frd.*f+Trd.*g;
kd=(K.*rd-bb.*rhs2)./detj; sd=(-cc.*rd+aa.*rhs2)./detj;
beta=ppval(tab.beta,k); bd=ppval(tab.betad,k);
q=theta+g.*s; h=q-beta;
qd=td+gd.*kd.*s+g.*sd; hd=qd-bd.*kd;
F=(1-fd./ld.*s).*Fr-gd./ld.*s.*Tr;
tau=[Tr,ld.*F+bd.*Tr]; dq=[hd,kd];
[lo,hi,rpm]=motor_envelope(dq,p);
[exactToe,qa,points]=eflm_geometry(h,k,s,p);
if ~isreal(exactToe)||any(~isfinite(exactToe),'all'), error('EFLM:Geometry','Nonreal elastic geometry'); end
qhks=1e-20;
[~,qak]=eflm_geometry(h,k+1i*qhks,s,p); [~,qas]=eflm_geometry(h,k,s+1i*qhks,p);
qad=imag(qak)/qhks.*kd+imag(qas)/qhks.*sd;
power=tau.*dq; energy=trapz(b.t,sum(abs(power),2));
[springForce,springStiffness,springEnergy]=spring_response(s,p);
J=zeros(2,2,n); J(1,2,:)=ld; J(2,1,:)=1; J(2,2,:)=bd;
cot=energy/(p.m*p.g*b.distance);
% Fixed-dimensional smooth inequality samples, c<=0. Speed-domain positivity included.
channels=[(tau-hi)/p.tauMax,(lo-tau)/p.tauMax,...
 [h,k]-p.jointUpper,p.jointLower-[h,k],...
 (s-p.smax)/p.smax,-s/p.smax,(abs(rpm)-p.rpmLimit)/p.rpmLimit,-b.z/p.leg];
durationConstraints=[(cfg.minimumStance-b.Ts)/cfg.minimumStance;...
 (cfg.minimumFlight-b.Tf)/cfg.minimumFlight];
c=[channels(:);-b.distance;durationConstraints];
envelope=[max(channels,[],1)';-b.distance;durationConstraints];
r=struct('a',a,'body',b,'s',s,'sd',sd,'l',l,'ld',ld.*kd,'q',q,'qd',qd,...
 'angles',[h,k,qa],'angularVelocity',[dq,qad],'tau',tau,'rpm',rpm,...
 'torqueLower',lo,'torqueUpper',hi,'power',power,'energy',energy,'cot',cot,...
 'objective',cot-10*b.speed,'constraints',c,'maxViolation',max([0;c]),...
 'constraintChannels',channels,'constraintEnvelope',envelope,...
 'durationConstraints',durationConstraints,...
 'equilibriumResidual',[e1,e2],'pointsLocal',points,...
 'jacobian',J,'Fr',Fr,'tauR',Tr,'F',F,'springForce',springForce,...
 'springStiffness',springStiffness,'springEnergy',springEnergy,...
 'cumulativeAbsoluteWork',cumtrapz(b.t,abs(power)),...
 'exactToeResidual',exactToe-Y,'iterations',it,'elapsed',toc(watch),...
 'success',true);
end
