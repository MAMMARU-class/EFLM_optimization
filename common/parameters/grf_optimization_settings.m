function [cfg,p]=grf_optimization_settings(kind,overrides)
% Layout 4 is shared by Hermite and open-uniform cubic B-spline GRFs.
cfg=optimization_settings(); p=robot_parameters();
cfg=rmfield(cfg,{'startsPerK','minimumStance','minimumFlight',...
 'grfXN','grfZN','grfXBetaStart','grfXBetaEnd','grfZBetaStart','grfZBetaEnd',...
 'restrictGrfK','grfKSelection'});
cfg.layoutVersion=4; cfg.curveType=kind; cfg.controlCount=11;
cfg.lower=[1 deg2rad(40) .26 .05 .3];
cfg.upper=[3.5 deg2rad(65) .33 .25 .6];
cfg.initial=[1 deg2rad(50) .30 .15 .5];
cfg.minGaitDuration=.25; cfg.broadStarts=500000; cfg.localStarts=10000;
cfg.save_interval=2000; cfg.narrowNodes=1201; cfg.parallelWorkers=20;
cfg.w1=5; cfg.w2=1; cfg.targetSpeed=4; cfg.targetCoT=.3;
cfg.model='stage1_point_mass';
if strcmp(kind,'bspline'), cfg.model='stage1_bspline'; end
cfg.method=['independent_stance_time_' kind];
basic={'vx0','thetaL','r0','stanceTime','duty'};
for j=1:numel(basic)
 name=basic{j};
 if ~isfield(overrides,name), continue; end
 v=overrides.(name); validateattributes(v,{'numeric'},{'row','numel',3,'real','finite'});
 cfg.lower(j)=v(1); cfg.upper(j)=v(2); cfg.initial(j)=v(3);
 overrides=rmfield(overrides,name);
end
protected={'lower','upper','initial','index','layoutVersion','curveType','model','method',...
 'stanceScale','minimumStance','minimumFlight','grfXN','grfZN','grfKSelection',...
 'restrictGrfK','grfXBetaStart','grfXBetaEnd','grfZBetaStart','grfZBetaEnd','startsPerK'};
for name=fieldnames(overrides)'
 if ~isfield(cfg,name{1}) || ismember(name{1},protected)
  error('EFLM:UnknownSetting','Unsupported setting for layout 4: %s',name{1});
 end
 cfg.(name{1})=overrides.(name{1});
end
for name={'controlCount','broadStarts','localStarts','save_interval','nodes','narrowNodes','finalNodes','parallelWorkers'}
 validateattributes(cfg.(name{1}),{'numeric'},{'scalar','finite','integer','positive'});
end
assert(cfg.controlCount>=4 && min([cfg.nodes cfg.narrowNodes cfg.finalNodes])>=3,'Invalid discretization.');
validateattributes(cfg.minGaitDuration,{'numeric'},{'scalar','finite','positive'});
if any(cfg.lower>=cfg.upper) || any(cfg.initial<cfg.lower | cfg.initial>cfg.upper) || ...
 cfg.lower(1)<=0 || cfg.lower(3)<=0 || cfg.lower(4)<=0 || cfg.lower(5)<=0 || cfg.upper(5)>=1 || ...
 cfg.upper(4)/cfg.lower(5)<cfg.minGaitDuration
 error('EFLM:Bounds','Invalid basic/timing bounds.');
end
if cfg.initial(4)/cfg.initial(5)<cfg.minGaitDuration
 error('EFLM:Bounds','Initial stanceTime/duty must meet minGaitDuration.');
end
n=cfg.controlCount-2;
cfg.index=struct('r0',3,'stanceTime',4,'fx',6:5+n,'fz',6+n:5+2*n);
cfg.variableNames=basic;
cfg.variableNames{4}='T_st';
for j=1:n, cfg.variableNames{end+1}=sprintf('FxControl%d',j); end
for j=1:n, cfg.variableNames{end+1}=sprintf('FzShapeControl%d',j); end
op=grf_operators(kind,cfg.controlCount,1); w=op.I(2:end-1);
assert(all(w>0),'Interior basis integrals must be positive.');
% Redundant finite box for normalized fmincon coordinates. The actual limit
% is ALWAYS |Fx_i|<=mu*Az*fz_i, and is not an independent force restriction.
bound=p.frictionCoefficient*p.m*p.g./(cfg.lower(5)*w);
cfg.lower=[cfg.lower,-bound,zeros(1,n)];
cfg.upper=[cfg.upper,bound,ones(1,n)];
cfg.initial=[cfg.initial,zeros(1,n),ones(1,n)/n];
cfg.constraintChannelCount=18;
cfg.shapeConstraintCount=2*n+3*(cfg.controlCount-1)*strcmp(kind,'hermite');
end
