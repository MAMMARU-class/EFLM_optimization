function cfg = configure_stage1_basic(options)
% Fixed basic coordinates are excluded from the local optimization vector.
names={'vx0','thetaL','r0'}; axes=cell(1,3); specs=zeros(3,3);
defaults={[2.2 .4 5],[deg2rad(50) deg2rad(10) 5],[.3 .05 5]};
for i=1:3
 name=names{i}; spec=defaults{i};
 if isfield(options,name), spec=options.(name); options=rmfield(options,name); end
 validateattributes(spec,{'numeric'},{'real','finite','size',[1 3]});
 validateattributes(spec(3),{'numeric'},{'integer','positive'});
 if mod(spec(3),2)==0
  error('EFLM:GridCount','%s: grid count must be odd so the center is included.',name);
 end
 if spec(2)<0 || (spec(3)>1 && spec(2)==0) || (spec(3)==1 && spec(2)~=0)
  error('EFLM:GridBounds','%s: use positive width for multiple points, or [center 0 1] for one point.',name);
 end
 axes{i}=linspace(spec(1)-spec(2),spec(1)+spec(2),spec(3)); specs(i,:)=spec;
end
if specs(1,1)-specs(1,2)<=0 || specs(3,1)-specs(3,2)<=0 || ...
   specs(2,1)-specs(2,2)<=0 || specs(2,1)+specs(2,2)>=pi/2
 error('EFLM:GridBounds','Require vx0>0, r0>0, and 0<thetaL<pi/2.');
end
topResults=3;
if isfield(options,'topResults'), topResults=options.topResults; options=rmfield(options,'topResults'); end
validateattributes(topResults,{'numeric'},{'scalar','integer','nonnegative','finite'});
for name={'localStarts','localRadius','finalNodes','layoutVersion','lower','upper','initial'}
 if isfield(options,name{1}), error('EFLM:BasicSetting','%s is not a stage1_basic setting.',name{1}); end
end
cfg=optimization_settings();
cfg=apply_stage1_overrides(cfg,options);
% These three bounds only describe grid coverage; they never enter fmincon.
cfg.lower(1:3)=(specs(:,1)-specs(:,2))';
cfg.upper(1:3)=(specs(:,1)+specs(:,2))';
cfg.initial(1:3)=specs(:,1)';
cfg=configure_impulse_grf(cfg);
cfg.gridAxes=axes; cfg.gridSpecs=specs; cfg.topResults=topResults;
cfg.model='stage1_basic'; cfg.simulationFunction='sim_cycle_stage1_basic';
cfg.freeIndices=setdiff(1:numel(cfg.initial),cfg.parameterGroups.basic);
cfg.method='basic_grid_broad_multistart';
cfg=rmfield(cfg,{'localStarts','localRadius','finalNodes','starts'});
validateattributes(cfg.nodes,{'numeric'},{'scalar','integer','>=',3});
validateattributes(cfg.seed,{'numeric'},{'scalar','integer','nonnegative','finite'});
if any(cfg.lower(cfg.freeIndices)>=cfg.upper(cfg.freeIndices)) || any(~isfinite([cfg.lower cfg.upper cfg.initial]))
 error('EFLM:Bounds','Continuous bounds must be finite with lower < upper.');
end
if any(cfg.initial<cfg.lower | cfg.initial>cfg.upper), error('EFLM:InitialPoint','Initial values must be within bounds.'); end
validateattributes(cfg.targetSpeed,{'numeric'},{'scalar','positive','finite'});
validateattributes(cfg.targetCoT,{'numeric'},{'scalar','positive','finite'});
validateattributes([cfg.w1 cfg.w2],{'numeric'},{'nonnegative','finite'});
if cfg.w1+cfg.w2<=0, error('EFLM:Weights','At least one objective weight must be positive.'); end
end
