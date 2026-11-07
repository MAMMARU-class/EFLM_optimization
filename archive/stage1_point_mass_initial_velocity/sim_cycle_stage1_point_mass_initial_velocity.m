function r = sim_cycle_stage1_point_mass_initial_velocity(a,p,tab,cfg,n)
% Massless exact EFLM, docs (19a). All stance nodes solved together.
watch=tic; b=prescribed_motion_initial_velocity(a,p,n); Y=[-b.x,-b.z]; V=[-b.vx,-b.vz];
G=-[b.Fx,b.Fz]; Gd=-[b.dFx,b.dFz];
rad=sqrt(sum(Y.^2,2)); rd=sum(Y.*V,2)./rad;
theta=atan2(-Y(:,2),Y(:,1)); td=(Y(:,2).*V(:,1)-Y(:,1).*V(:,2))./rad.^2;
Fr=sum(Y.*G,2)./rad; Tr=Y(:,2).*G(:,1)-Y(:,1).*G(:,2);
Frd=(sum(V.*G+Y.*Gd,2)-Fr.*rd)./rad;
Trd=V(:,2).*G(:,1)+Y(:,2).*Gd(:,1)-V(:,1).*G(:,2)-Y(:,1).*Gd(:,2);
k=ppval(tab.inverse,min(tab.lRange(2),max(tab.lRange(1),rad)));
s=zeros(n,1); converged=false; epsc=1e-20;
for it=1:60
 d=exact_polar_geometry_initial_velocity(k,s,p); [Fs,K]=spring_response(s,p);
 e1=d.R-rad; e2=Fs+Fr.*d.Rs+Tr.*d.Bs;
 if any(~isfinite(e1+e2)), error('EFLM:Geometry','Invalid exact closure'); end
 if max(abs(e1))<1e-13 && max(abs(e2))<1e-10, converged=true; break; end
 [aa,bb,cc,dd,detj]=systemJacobian(d,k,s,K);
 dk=(dd.*e1-bb.*e2)./detj; ds=(-cc.*e1+aa.*e2)./detj;
 factor=1./max(1,max(abs(dk)/.10,abs(ds)/.003));
 merit=(e1/p.lengthScale).^2+(e2/(p.ks*p.smax)).^2;
 for backtrack=1:24
  kt=k-factor.*dk; st=s-factor.*ds; dt=exact_polar_geometry_initial_velocity(kt,st,p);
  et1=dt.R-rad; et2=spring_response(st,p)+Fr.*dt.Rs+Tr.*dt.Bs;
  trial=(et1/p.lengthScale).^2+(et2/(p.ks*p.smax)).^2;
  bad=~isfinite(trial) | kt<tab.kRange(1) | kt>tab.kRange(2) ...
   | (trial>merit & merit>1e-22);
  if ~any(bad), break; end
  factor(bad)=factor(bad)/2;
 end
 if any(bad), error('EFLM:Convergence','Exact Newton line search failed'); end
 k=kt; s=st;
 if toc(watch)>cfg.evaluationSeconds, error('EFLM:Timeout','Evaluation exceeded time budget'); end
end
if ~converged, error('EFLM:Convergence','Exact equilibrium did not converge'); end
% Enforce the unilateral stop at the preloaded reference length.
atStop=s<0;
if any(atStop)
 s(atStop)=0;
 k(atStop)=ppval(tab.inverse,rad(atStop));
 for j=1:4
  dt=exact_polar_geometry_initial_velocity(k(atStop),s(atStop),p);
  k(atStop)=k(atStop)-(dt.R-rad(atStop))./dt.Rk;
 end
 d=exact_polar_geometry_initial_velocity(k,s,p); [Fs,K]=spring_response(s,p);
 e1=d.R-rad; e2=Fs+Fr.*d.Rs+Tr.*d.Bs;
end
stopReaction=zeros(n,1);
stopReaction(atStop)=e2(atStop);
if any(stopReaction< -1e-8), error('EFLM:Stop','Invalid preload stop reaction'); end
e2(atStop)=e2(atStop)-stopReaction(atStop);
[aa,bb,cc,dd,detj]=systemJacobian(d,k,s,K);
rhs2=-Frd.*d.Rs-Trd.*d.Bs;
kd=(dd.*rd-bb.*rhs2)./detj; sd=(-cc.*rd+aa.*rhs2)./detj;
kd(atStop)=rd(atStop)./d.Rk(atStop); sd(atStop)=0;
h=theta-d.beta; hd=td-d.Bk.*kd-d.Bs.*sd;
tau=[Tr,d.Rk.*Fr+d.Bk.*Tr]; dq=[hd,kd];
qad=d.qak.*kd+d.qas.*sd;
% Virtual reference coordinates retain the same meanings as approx exports.
ref=exact_polar_geometry_initial_velocity(k,zeros(n,1),p);
l=ref.R; q=h+ref.beta; qd=hd+ref.Bk.*kd;
F=(tau(:,2)-ref.Bk.*Tr)./ref.Rk;
J=zeros(2,2,n); J(1,2,:)=ref.Rk; J(2,1,:)=1; J(2,2,:)=ref.Bk;
[toe,qa,points]=eflm_geometry(h,k,s,p);
if ~isreal(toe)||any(~isfinite(toe),'all'), error('EFLM:Geometry','Nonreal exact geometry'); end
[lo,hi,rpm]=motor_envelope(dq,p);
power=tau.*dq; energy=trapz(b.t,sum(abs(power),2)); cot=energy/(p.m*p.g*b.distance);
channels=[(tau-hi)/p.tauMax,(lo-tau)/p.tauMax,...
 [h,k]-p.jointUpper,p.jointLower-[h,k],...
 (s-p.smax)/p.smax,-s/p.smax,(abs(rpm)-p.rpmLimit)/p.rpmLimit,-b.z/p.lengthScale];
durationConstraints=[(cfg.minimumStance-b.Ts)/cfg.minimumStance;...
 (cfg.minimumFlight-b.Tf)/cfg.minimumFlight];
c=[channels(:);-b.distance;durationConstraints];
[springForce,springStiffness,springEnergy]=spring_response(s,p);
r=struct('a',a,'body',b,'s',s,'sd',sd,'l',l,'ld',ref.Rk.*kd,'q',q,'qd',qd,...
 'angles',[h,k,qa],'angularVelocity',[dq,qad],'tau',tau,'rpm',rpm,...
 'torqueLower',lo,'torqueUpper',hi,'power',power,'energy',energy,'cot',cot,...
 'objective',stage1_target_cost(b.speed,cot,cfg),'constraints',c,'maxViolation',max([0;c]),...
 'constraintChannels',channels,'constraintEnvelope',[max(channels,[],1)';-b.distance;durationConstraints],...
 'durationConstraints',durationConstraints,'equilibriumResidual',[e1,e2],...
 'stopReaction',stopReaction,...
 'pointsLocal',points,'jacobian',J,'Fr',Fr,'tauR',Tr,'F',F,...
 'springForce',springForce,'springStiffness',springStiffness,'springEnergy',springEnergy,...
 'cumulativeAbsoluteWork',cumtrapz(b.t,abs(power)),'exactToeResidual',toe-Y,...
 'iterations',it,'elapsed',toc(watch),'success',true);

 function [aa,bb,cc,dd,detj]=systemJacobian(d,k,s,K)
  dkgeom=exact_polar_geometry_initial_velocity(k+1i*epsc,s,p);
  dsgeom=exact_polar_geometry_initial_velocity(k,s+1i*epsc,p);
  aa=d.Rk; bb=d.Rs;
  cc=Fr.*imag(dkgeom.Rs)/epsc+Tr.*imag(dkgeom.Bs)/epsc;
  dd=K+Fr.*imag(dsgeom.Rs)/epsc+Tr.*imag(dsgeom.Bs)/epsc;
  detj=aa.*dd-bb.*cc;
  if any(~isfinite(detj)|abs(detj)<1e-6), error('EFLM:Singular','Exact Jacobian singular'); end
 end
end
