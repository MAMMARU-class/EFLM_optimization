function record = optimize_basic_case(basic,k,caseId,p,tab,cfg)
% One broad MultiStart at a fixed (vx0, thetaL, r0, k).
watch=tic; cfg.grfK=k; cfg.initial(cfg.parameterGroups.basic)=basic;
seed=mod(cfg.seed+caseId-1,2^32); rng(seed,'twister');
idx=cfg.freeIndices; lo=cfg.lower(idx); width=cfg.upper(idx)-lo;
if isfield(cfg,'adaptiveLocal') && cfg.adaptiveLocal
 physical=basic_bounded_starts(cfg,basic);
else
 physical=cfg.lower+rand(cfg.startsPerK,numel(cfg.initial)).*(cfg.upper-cfg.lower);
 physical(:,cfg.parameterGroups.basic)=repmat(basic,cfg.startsPerK,1);
 physical=seed_horizontal_impulse(physical,cfg,true);
 zi=cfg.index.fz; physical(:,zi)=physical(:,zi)./sum(physical(:,zi),2);
 physical(1,:)=seed_horizontal_impulse(cfg.initial,cfg,false);
end
starts=(physical(:,idx)-lo)./width;
queue=[]; if cfg.useParallel, queue=parallel.pool.PollableDataQueue; end
lastX=[]; lastR=[]; bestX=[]; bestCost=Inf; bestViolation=Inf;
iterations=0; evaluations=0; firstError=''; firstMessage='';
options=optimoptions('fmincon','Algorithm',cfg.algorithm,'Display','off',...
 'MaxIterations',cfg.maxIterations,'MaxFunctionEvaluations',cfg.maxEvaluations,...
 'ConstraintTolerance',cfg.constraintTolerance,'StepTolerance',cfg.stepTolerance,...
 'FiniteDifferenceStepSize',1e-6,'OutputFcn',@finishStart);
% A useful seed must survive even if the local solver terminates badly.
r=evaluate(starts(1,:)); accept(pack(starts(1,:),r,0,0));
problem=createOptimProblem('fmincon','x0',starts(1,:),'objective',@objective,...
 'nonlcon',@constraints,'lb',zeros(size(lo)),'ub',ones(size(lo)),'options',options);
ms=MultiStart('Display','off','UseParallel',cfg.useParallel,'StartPointsToRun','all','MaxTime',Inf);
[x,~,exitflag,output]=run(ms,problem,CustomStartPointSet(starts));
if cfg.useParallel
 while true
  msg=poll(queue,.1); if isempty(msg), break; end
  accept(msg);
 end
end
if ~isempty(x)
 r=evaluate(x); accept(pack(x,r,0,0));
end
record=struct('caseId',caseId,'basic',basic,'k',k,'seed',seed,...
 'status','no_evaluable_solution','feasible',false,'variables',[],...
 'cost',NaN,'maxViolation',NaN,'speed',NaN,'cot',NaN,'energy',NaN,...
 'distance',NaN,'stanceTime',NaN,'flightTime',NaN,'vz0',NaN,...
 'exitflag',exitflag,'starts',output.localSolverTotal,'convergedStarts',output.localSolverSuccess,...
 'iterations',iterations,'functionEvaluations',evaluations,'elapsed',toc(watch),...
 'errorIdentifier',firstError,'errorMessage',firstMessage);
if ~isempty(bestX)
 r=evaluate(bestX); a=expand(bestX);
 record.variables=a;
 if ~r.success
  record.errorIdentifier=r.errorIdentifier; record.errorMessage=r.errorMessage;
  record.elapsed=toc(watch); return;
 end
 record.cost=r.objective; record.maxViolation=r.maxViolation;
 record.speed=r.body.speed; record.cot=r.cot; record.energy=r.energy;
 record.distance=r.body.distance; record.stanceTime=r.body.Ts; record.flightTime=r.body.Tf; record.vz0=r.body.vz0;
 record.feasible=r.maxViolation<=cfg.constraintTolerance;
 record.status='no_feasible_solution_found';
 if record.feasible, record.status='feasible_at_evaluation_nodes'; end
 record.errorIdentifier=''; record.errorMessage='';
end
record.elapsed=toc(watch);

 function a=expand(x)
  a=cfg.initial; a(idx)=lo+reshape(x,1,[]).*width;
 end
 function r=evaluate(x)
  x=reshape(x,1,[]);
  if isequal(x,lastX), r=lastR; return; end
  try
   r=sim_cycle_stage1_basic(expand(x),p,tab,cfg,cfg.nodes);
   if r.body.distance<=0 || ~isfinite(r.objective), error('EFLM:Distance','Invalid travel or objective.'); end
  catch ME
   if ~startsWith(ME.identifier,'EFLM:'), rethrow(ME); end
   r=struct('success',false,'objective',cfg.penalty+sum(x.^2),'maxViolation',Inf,...
    'constraintEnvelope',ones(20+cfg.shapeConstraintCount,1),'equalityConstraints',ones(3,1),...
    'errorIdentifier',ME.identifier,'errorMessage',ME.message);
  end
  lastX=x; lastR=r;
 end
 function f=objective(x), r=evaluate(x); f=r.objective; end
 function [c,ceq]=constraints(x), r=evaluate(x); c=r.constraintEnvelope; ceq=r.equalityConstraints; end
 function msg=pack(x,r,it,ev)
  msg=struct('x',reshape(x,1,[]),'success',r.success,'cost',r.objective,...
   'violation',r.maxViolation,'iterations',it,'evaluations',ev,'errorIdentifier','','errorMessage','');
  if ~r.success, msg.errorIdentifier=r.errorIdentifier; msg.errorMessage=r.errorMessage; end
 end
 function stop=finishStart(x,values,state)
  stop=false; if ~strcmp(state,'done'), return; end
  r=evaluate(x); msg=pack(x,r,values.iteration,values.funccount);
  if cfg.useParallel && ~isempty(getCurrentTask()), send(queue,msg); else, accept(msg); end
 end
 function accept(msg)
  iterations=iterations+msg.iterations; evaluations=evaluations+msg.evaluations;
  if ~msg.success
   if isempty(firstError), firstError=msg.errorIdentifier; firstMessage=msg.errorMessage; end
   return;
  end
  feasible=msg.violation<=cfg.constraintTolerance; incumbent=bestViolation<=cfg.constraintTolerance;
  if isempty(bestX) || (feasible && ~incumbent) || ...
    (feasible && incumbent && msg.cost<bestCost) || ...
    (~feasible && ~incumbent && msg.violation<bestViolation)
   bestX=msg.x; bestCost=msg.cost; bestViolation=msg.violation;
  end
 end
end
