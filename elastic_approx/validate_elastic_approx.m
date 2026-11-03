function report = validate_elastic_approx()
root=fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root,'common')),genpath(fullfile(root,'elastic_approx')));
p=robot_parameters(); cfg=optimization_settings(); tab=build_geometry_cache(p);
% Check smooth force slope, exact force/energy consistency on positive domain.
s=[1e-5 3e-5 8e-5 .003]'; step=1e-9;
[~,K]=spring_response(s,p); [fp,~,up]=spring_response(s+step,p); [fm,~,um]=spring_response(s-step,p);
F=spring_response(s,p);
assert(max(abs((fp-fm)/(2*step)-K)./K)<1e-6);
assert(max(abs((up-um)/(2*step)-F))<1e-5);
r=sim_cycle_elastic_approx(cfg.initial,p,tab,cfg,301);
assert(max(abs(r.body.periodic))<1e-10);
assert(max(abs(r.equilibriumResidual(:,1)))<1e-9);
assert(max(abs(r.equilibriumResidual(:,2)))<1e-6);
% Independent finite-difference rates, excluding endpoints.
dh=gradient(r.angles(:,1),r.body.t); dk=gradient(r.angles(:,2),r.body.t);
report=struct('initialObjective',r.objective,'initialViolation',r.maxViolation,...
 'rateError',max(abs([dh dk]-r.angularVelocity(:,1:2)),[],'all'),...
 'algebraicResidual',max(abs(r.equilibriumResidual),[],1),...
 'periodic',r.body.periodic,'elapsed',r.elapsed);
disp(report);
% Virtual work identity includes stored spring energy, with no double counting.
Yd=[-r.body.vx,-r.body.vz]; G=-[r.body.Fx,r.body.Fz];
assert(max(abs(sum(r.power,2)-sum(G.*Yd,2)-spring_response(r.s,p).*r.sd))<1e-7);
% Geometry interpolants versus direct closed-chain evaluation, away from knots.
k=deg2rad([23.123;61.234;101.345;137.456]);
[Y,qa]=eflm_geometry(zeros(size(k)),k,zeros(size(k)),p);
assert(max(abs(ppval(tab.l,k)-sqrt(sum(Y.^2,2))))<1e-9);
assert(max(abs(ppval(tab.qa,k)-qa))<1e-8);
assert(numel(r.constraints)==13*numel(r.body.t)+3);
fine=sim_cycle_elastic_approx(cfg.initial,p,tab,cfg,12001);
df=[gradient(fine.angles(:,1),fine.body.t),gradient(fine.angles(:,2),fine.body.t)];
report.fineRateError=max(abs(df-fine.angularVelocity(:,1:2)),[],'all');
% Extremely clustered endpoint differences suffer cancellation; test rates
% independently in the resolved interior, and report endpoint errors separately.
interior=fine.s>2*p.rise & fine.body.t>.01*fine.body.Ts & fine.body.t<.99*fine.body.Ts;
report.interiorRateError=max(abs(df(interior,:)-fine.angularVelocity(interior,1:2)),[],'all');
assert(report.interiorRateError<1e-3,'Implicit joint rates disagree with time differences');
assert(r.maxViolation<=cfg.constraintTolerance,'Default initial point must be feasible');
report.coarseCoT=r.cot; report.fineCoT=fine.cot;
disp(report);
end
