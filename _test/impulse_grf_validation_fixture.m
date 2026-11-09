function [cfg,p,tab] = impulse_grf_validation_fixture()
% Translate one saved trajectory into a reproducible new-layout test seed.
root=fileparts(fileparts(mfilename('fullpath')));
s=load(fullfile(root,'_logs','stage1_point_mass','20261108_184726_939','result.mat'),'result');
old=s.result; h=old.final.body.horizontalGRF; z=old.final.body.verticalGRF;
cfg=optimization_settings(); cfg.grfXN=numel(h.values); cfg.grfZN=numel(z.values);
cfg.restrictGrfK=true; cfg.grfKSelection=h.k;
cfg=configure_impulse_grf(cfg); ix=cfg.index; a=cfg.initial;
v=old.bestVariables; a(1:6)=[v(3),v(2),v(5),1,v(4),trapz(old.final.body.t,abs(old.final.body.Fx))];
scale=sum(abs(h.values)); nodes=setdiff(2:cfg.grfXN-1,h.k);
a(ix.fx)=abs(h.values(nodes))/scale;
a(ix.betaXStart)=h.beta(1)/scale; a(ix.betaXEnd)=h.beta(end)/scale;
a(ix.fz)=z.values(2:end-1); a(ix.betaZStart)=z.beta(1); a(ix.betaZEnd)=z.beta(end);
p=robot_parameters(); b=prescribed_motion_stage1(a,p,121,cfg);
a(ix.stanceScale)=old.final.body.Ts/b.TsBase;
cfg.initial=a; cfg.simulationFunction='sim_cycle_stage1_point_mass';
tab=build_exact_geometry_cache(p);
end
