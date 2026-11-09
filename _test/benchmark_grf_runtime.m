function results = benchmark_grf_runtime()
% Short, representative MultiStart timing with the same solver and 16 workers.
root=fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root,'common')),fullfile(root,'stage1_point_mass'));
p=robot_parameters(); tab=build_exact_geometry_cache(p);
results=zeros(0,6); sampleStarts=64;
for n=[19 20 21]
 cfg=optimization_settings();
 override=struct('grfXN',n,'startsPerK',sampleStarts,'useParallel',true,'parallelWorkers',16);
 cfg=configure_impulse_grf(apply_stage1_overrides(cfg,override));
 prepare_optimization_pool(cfg);
 lo=cfg.lower; width=cfg.upper-lo; nvars=numel(lo);
 options=optimoptions('fmincon','Algorithm',cfg.algorithm,'Display','off',...
  'MaxIterations',cfg.maxIterations,'MaxFunctionEvaluations',cfg.maxEvaluations,...
  'ConstraintTolerance',cfg.constraintTolerance,'StepTolerance',cfg.stepTolerance,...
  'FiniteDifferenceStepSize',1e-6);
 ms=MultiStart('Display','off','UseParallel',true,'StartPointsToRun','all','MaxTime',Inf);
 rng(cfg.seed,'twister');
 kList=unique([3,floor(n/3),ceil(n/2),n-2]);
 for group=1:numel(kList)
  cfg.grfK=kList(group);
  candidate=lo+rand(sampleStarts,nvars).*width;
  candidate=seed_horizontal_impulse(candidate,cfg,true);
  zi=cfg.index.fz; candidate(:,zi)=candidate(:,zi)./sum(candidate(:,zi),2);
  candidate(1,:)=seed_horizontal_impulse(cfg.initial,cfg,false);
  starts=(candidate-lo)./width;
  problem=createOptimProblem('fmincon','x0',starts(1,:),...
   'objective',@objective,'nonlcon',@constraints,...
   'lb',zeros(1,nvars),'ub',ones(1,nvars),'options',options);
  lastX=[]; lastR=[]; watch=tic;
  [~,~,~,output,~]=run(ms,problem,CustomStartPointSet(starts));
  elapsed=toc(watch);
  row=[n,kList(group),sampleStarts,elapsed,elapsed/sampleStarts,output.localSolverSuccess];
  results(end+1,:)=row; %#ok<AGROW>
  fprintf('n=%d k=%d starts=%d elapsed=%.3f s (%.4f s/start)\n',row(1:5));
 end
end
disp(array2table(results,'VariableNames',{'n','k','starts','elapsedSeconds','secondsPerStart','successfulStarts'}));

 function r=evaluate(x)
  if isequal(x,lastX), r=lastR; return; end
  a=lo+x.*width;
  try
   r=sim_cycle_stage1_point_mass(a,p,tab,cfg,cfg.nodes);
   if r.body.distance<=0 || ~isfinite(r.objective), error('EFLM:Distance','Nonpositive travel'); end
  catch ME
   if ~startsWith(ME.identifier,'EFLM:'), rethrow(ME); end
   r=struct('objective',cfg.penalty+sum(x.^2),...
    'constraintEnvelope',ones(20+cfg.shapeConstraintCount,1),'equalityConstraints',ones(cfg.equalityConstraintCount,1));
  end
  lastX=x; lastR=r;
 end
 function f=objective(x), r=evaluate(x); f=r.objective; end
 function [c,ceq]=constraints(x), r=evaluate(x); c=r.constraintEnvelope; ceq=r.equalityConstraints; end
end
