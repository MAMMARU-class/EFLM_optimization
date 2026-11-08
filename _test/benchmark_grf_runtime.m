function results = benchmark_grf_runtime()
% Short, representative MultiStart timing with the same solver and 16 workers.
root=fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root,'common')),fullfile(root,'stage1_point_mass'));
p=robot_parameters(); tab=build_exact_geometry_cache(p);
results=zeros(0,7); sampleStarts=64;
for n=[19 20 21]
 cfg=optimization_settings();
 override=struct('AmpH',[1 35 10],'thetaL',deg2rad([30 65 50]),...
  'v',[.4 4 1],'Duty',[.1 .6 .1],'r0',[.25 .33 .30],...
  'stanceScale',[.5 2 1],'grfN',n,'grfBetaEndpoint',[-.4 0 0],...
  'starts',max(sampleStarts,(n-4)*(n-3)),'useParallel',true,'parallelWorkers',16);
 cfg=configure_horizontal_grf(apply_stage1_overrides(cfg,override));
 prepare_optimization_pool(cfg);
 lo=cfg.lower; width=cfg.upper-lo; nvars=numel(lo);
 options=optimoptions('fmincon','Algorithm',cfg.algorithm,'Display','off',...
  'MaxIterations',cfg.maxIterations,'MaxFunctionEvaluations',cfg.maxEvaluations,...
  'ConstraintTolerance',cfg.constraintTolerance,'StepTolerance',cfg.stepTolerance,...
  'FiniteDifferenceStepSize',1e-6);
 ms=MultiStart('Display','off','UseParallel',true,'StartPointsToRun','all','MaxTime',Inf);
 rng(cfg.seed,'twister');
 pairList=[3,2;3,n-1;floor(n/3),2;ceil(n/2),2;...
  ceil(n/2),ceil(n/2)+1;n-2,n-1];
 for pair=1:size(pairList,1)
  cfg.grfK=pairList(pair,1); cfg.grfJ=pairList(pair,2);
  candidate=[cfg.initial;lo+rand(sampleStarts-1,nvars).*width];
  count=floor((sampleStarts-1)/2);
  angles=deg2rad(1+8*rand(count,1));
  ts=cfg.minimumStance+.02*rand(count,1); tf=cfg.minimumFlight+.02*rand(count,1);
  lengths=lo(5)+width(5)*rand(count,1);
  amplitudes=min(cfg.upper(1),max(lo(1),rand(count,1)));
  scales=lo(6)+width(6)*rand(count,1);
  velocities=scales.*2.*lengths.*sin(angles)./ts;
  candidate(2:count+1,1:6)=[amplitudes,angles,velocities,ts./(ts+tf),lengths,scales];
  candidate(2:count+1,7:end)=repmat(cfg.initial(7:end),count,1);
  values=.5*ones(1,n); values([1 cfg.grfK end])=0; values(cfg.grfJ)=1;
  negative=2:cfg.grfK-1; positive=cfg.grfK+1:n-1;
  if cfg.grfJ<cfg.grfK, same=negative; other=positive;
  else, same=positive; other=negative; end
  if ~isempty(other), values(other)=min(.95,sum(values(same))/numel(other)); end
  ratioNodes=setdiff(2:n-1,[cfg.grfK cfg.grfJ]);
  candidate(1,7:6+n-4)=values(ratioNodes);
  starts=(candidate-lo)./width;
  problem=createOptimProblem('fmincon','x0',starts(1,:),...
   'objective',@objective,'nonlcon',@constraints,...
   'lb',zeros(1,nvars),'ub',ones(1,nvars),'options',options);
  lastX=[]; lastR=[]; watch=tic;
  [~,~,~,output,~]=run(ms,problem,CustomStartPointSet(starts));
  elapsed=toc(watch);
  row=[n,pairList(pair,:),sampleStarts,elapsed,elapsed/sampleStarts,output.localSolverSuccess];
  results(end+1,:)=row; %#ok<AGROW>
  fprintf('n=%d k=%d j=%d starts=%d elapsed=%.3f s (%.4f s/start)\n',row(1:6));
 end
end
disp(array2table(results,'VariableNames',{'n','k','j','starts','elapsedSeconds','secondsPerStart','successfulStarts'}));

 function r=evaluate(x)
  if isequal(x,lastX), r=lastR; return; end
  a=lo+x.*width;
  try
   r=sim_cycle_stage1_point_mass(a,p,tab,cfg,cfg.nodes);
   if r.body.distance<=0 || ~isfinite(r.objective), error('EFLM:Distance','Nonpositive travel'); end
  catch ME
   if ~startsWith(ME.identifier,'EFLM:'), rethrow(ME); end
   r=struct('objective',cfg.penalty+sum(x.^2),...
    'constraintEnvelope',ones(19+cfg.shapeConstraintCount,1),'equalityConstraints',1);
  end
  lastX=x; lastR=r;
 end
 function f=objective(x), r=evaluate(x); f=r.objective; end
 function [c,ceq]=constraints(x), r=evaluate(x); c=r.constraintEnvelope; ceq=r.equalityConstraints; end
end
