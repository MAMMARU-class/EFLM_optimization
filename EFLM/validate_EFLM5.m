function metrics = validate_EFLM5()
root=fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root,'common')),genpath(fullfile(root,'EFLM')));
p=robot_parameters; cfg=optimization_settings; tab=build_eflm5_cache(p);
r=sim_cycle_EFLM5(cfg.initial,p,tab,cfg,12001); b=r.body;
metrics.contact=max(abs(r.contactResidual),[],'all');
metrics.lagrange=max(abs(r.lagrangeResidual),[],'all');
% Independent centered finite differences of potential and foot geometry.
idx=round(linspace(100,11900,31)); q=r.angles(idx,:); G=[b.Fx(idx),b.Fz(idx)];
d=eflm5_geometry(q,p); numerical=zeros(size(q)); epsfd=1e-6;
for j=1:3
 qp=q; qm=q; qp(:,j)=qp(:,j)+epsfd; qm(:,j)=qm(:,j)-epsfd;
 dp=eflm5_geometry(qp,p); dm=eflm5_geometry(qm,p);
 numerical(:,j)=(.5*p.ks*(dp.s.^2-dm.s.^2)-sum((dp.toe-dm.toe).*G,2))/(2*epsfd);
end
[~,balance]=eflm5_balance(q,-[b.x(idx),b.z(idx)],G,p);
metrics.independentTorque=max(abs(numerical-balance.extraTorque-[r.tau(idx,:),zeros(numel(idx),1)]),[],'all');
% Translational kinetic+gravity energy rate plus linear spring energy rate.
% urls' extra torque performs work separately from U (do not hide it).
powerBalance=sum(r.power,2)+r.additionalSpringPower...
 -(b.Fx.*b.vx+b.Fz.*b.vz+p.ks*r.s.*r.sd);
metrics.power=max(abs(powerBalance));
rates=zeros(size(r.angles));
for j=1:3, rates(:,j)=gradient(r.angles(:,j),b.t); end
metrics.rate=max(abs(rates(100:end-100,:)-r.angularVelocity(100:end-100,:)),[],'all');
% Short independent time perturbations avoid the narrow preload transition
% being under-resolved by the exported trajectory's finite-difference stencil.
dt=1e-7; shifted=cell(1,2); Y=-[b.x(idx),b.z(idx)]; V=-[b.vx(idx),b.vz(idx)];
Gd=[b.dFx(idx),b.dFz(idx)];
for side=1:2
 shift=(2*side-3)*dt; qs=q; Ys=Y+shift*V; Gs=G+shift*Gd;
 for iteration=1:5
  e=eflm5_balance(qs,Ys,Gs,p); J=eflm5_jacobian(qs,Ys,Gs,p);
  qs=qs-reshape(pagemldivide(J,reshape(e',3,1,numel(idx))),3,numel(idx))';
 end
 shifted{side}=qs;
end
metrics.implicitRate=max(abs((shifted{2}-shifted{1})/(2*dt)-r.angularVelocity(idx,:)),[],'all');
assert(metrics.contact<1e-10 && metrics.lagrange<1e-8);
assert(metrics.independentTorque<1e-5 && metrics.power<1e-8 && metrics.implicitRate<1e-4);
counts=[121,1201]; metrics.seconds=zeros(1,2);
for j=1:2
 sim_cycle_EFLM5(cfg.initial,p,tab,cfg,counts(j)); timer=tic;
 for i=1:5, sim_cycle_EFLM5(cfg.initial,p,tab,cfg,counts(j)); end
 metrics.seconds(j)=toc(timer)/5;
end
disp(metrics);
end
