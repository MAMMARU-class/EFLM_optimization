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
end
