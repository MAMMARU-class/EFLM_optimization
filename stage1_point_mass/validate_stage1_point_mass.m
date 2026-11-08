function report = validate_stage1_point_mass()
% Independent checks against Cartesian closed-link geometry and virtual work.
root=fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root,'common')),genpath(fullfile(root,'stage1_point_mass')));
p=robot_parameters(); cfg=configure_horizontal_grf(optimization_settings()); tab=build_exact_geometry_cache(p);
cfg.initial(7:9)=[.5 .75 .75]; % Balanced shape for n=7, k=4, j=2.
r=sim_cycle_stage1_point_mass(cfg.initial,p,tab,cfg,12001);
h=r.angles(:,1); k=r.angles(:,2); s=r.s; e=1e-20;
G=-[r.body.Fx,r.body.Fz]; V=-[r.body.vx,r.body.vz];
Yh=imag(eflm_geometry(h+1i*e,k,s,p))/e;
Yk=imag(eflm_geometry(h,k+1i*e,s,p))/e;
Ys=imag(eflm_geometry(h,k,s+1i*e,p))/e;
torqueError=max(abs(r.tau-[sum(G.*Yh,2),sum(G.*Yk,2)]),[],'all');
balanceError=max(abs(r.springForce+sum(G.*Ys,2)-r.stopReaction));
powerError=max(abs(sum(r.power,2)-sum(G.*V,2)-r.springForce.*r.sd));
toeError=max(abs(r.exactToeResidual),[],'all');
velocityError=max(abs(Yh.*r.angularVelocity(:,1)+Yk.*r.angularVelocity(:,2)+Ys.*r.sd-V),[],'all');
assert(torqueError<1e-9 && balanceError<1e-8 && powerError<1e-8);
assert(toeError<1e-10 && velocityError<1e-8);
assert(max(abs(r.body.periodic))<1e-10);
% Time finite differences independently check the implicit velocity solve.
mask=s>1e-4 & r.body.t>.01*r.body.Ts & r.body.t<.99*r.body.Ts;
numeric=[gradient(h,r.body.t),gradient(k,r.body.t),gradient(r.angles(:,3),r.body.t)];
rateError=max(abs(numeric(mask,:)-r.angularVelocity(mask,:)),[],'all');
assert(rateError<1e-3,'Exact implicit rates disagree with time differences');
% Benchmark the retained point-mass evaluator at the two mesh sizes.
times=zeros(2,1);
for j=1:2
 nodes=[121 1201]; n=nodes(j);
 sim_cycle_stage1_point_mass(cfg.initial,p,tab,cfg,n);
 timer=tic;
 for i=1:10, sim_cycle_stage1_point_mass(cfg.initial,p,tab,cfg,n); end
 times(j,1)=toc(timer)/10;
end
report=struct('toeError',toeError,'torqueError',torqueError,'balanceError',balanceError,...
 'powerError',powerError,'velocityError',velocityError,'rateError',rateError,...
 'timingNodes',[121;1201],'timingPointMassSeconds',times);
disp(report);
end
