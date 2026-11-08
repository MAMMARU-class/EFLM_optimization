function result = optimize_two_stage(p,tab,cfg)
% MultiStart global exploration followed by a smaller-box MultiStart refinement.
rng(cfg.seed,'twister');
prepare_optimization_pool(cfg);
simulate=str2func(cfg.simulationFunction);
messages=[];
if cfg.useParallel, messages=parallel.pool.PollableDataQueue; end
nvars=numel(cfg.initial); lastX=[]; lastR=[];
stageIterations=zeros(1,3); stageFunctionEvaluations=zeros(1,3);
stage=0; started=tic; stageClock=tic;
lo=cfg.lower; hi=cfg.upper; width=hi-lo;
options=optimoptions('fmincon','Algorithm',cfg.algorithm,'Display','off',...
 'MaxIterations',cfg.maxIterations,'MaxFunctionEvaluations',cfg.maxEvaluations,...
 'ConstraintTolerance',cfg.constraintTolerance,'StepTolerance',cfg.stepTolerance,...
 'FiniteDifferenceStepSize',1e-6,'OutputFcn',@recordSolverStats);
ms=MultiStart('Display','iter','UseParallel',cfg.useParallel,'StartPointsToRun','all',...
 'MaxTime',Inf);
stage=1; stageClock=tic;
x0=(cfg.initial-lo)./width;
starts=[x0;rand(cfg.starts-1,nvars)];
% Half of the starts sample plausible postures, retaining broad-box starts.
% This is random MultiStart seeding, never a parameter grid search.
count=floor((cfg.starts-1)/2);
if count>0
 angles=deg2rad(1+8*rand(count,1));
 ts=cfg.minimumStance+.02*rand(count,1); tf=cfg.minimumFlight+.02*rand(count,1);
 lengths=lo(5)+(hi(5)-lo(5))*rand(count,1);
 amplitudes=min(hi(1),max(lo(1),rand(count,1)));
 scales=lo(6)+(hi(6)-lo(6))*rand(count,1);
 velocities=scales.*2.*lengths.*sin(angles)./ts;
 seeds=[amplitudes,angles,velocities,ts./(ts+tf),lengths,scales];
 starts(2:count+1,1:6)=min(1,max(0,(seeds-lo(1:6))./width(1:6)));
 starts(2:count+1,7:end)=repmat(x0(7:end),count,1);
end
problem=createOptimProblem('fmincon','x0',x0,'objective',@objective,...
 'nonlcon',@constraints,'lb',zeros(1,nvars),'ub',ones(1,nvars),'options',options);
% Discrete knot choices stay fixed during each smooth local optimization.
% Distribute the requested total starts over the finite categorical choices.
pairs=cfg.grfPairs; numberPairs=size(pairs,1);
if cfg.starts<numberPairs
 error('EFLM:GRFStarts','starts must be at least %d to visit every (k,j) pair.',numberPairs);
end
pairIndex=mod((0:cfg.starts-1)',numberPairs)+1;
xg=[]; fg=Inf; bestViolation=Inf; distinctGlobalSolutions=0;
og=struct('localSolverTotal',0,'localSolverSuccess',0);
selectedPair=pairs(1,:);
for group=1:numberPairs
 cfg.grfK=pairs(group,1); cfg.grfJ=pairs(group,2); lastX=[]; lastR=[];
 indices=find(pairIndex==group);
 % Give each category a geometric initial point as well as random starts.
 % Balance knot magnitudes where possible; interior slopes follow the knots.
 seed=cfg.initial; values=.5*ones(1,cfg.grfN); values([1 cfg.grfK end])=0;
 values(cfg.grfJ)=1;
 negative=2:cfg.grfK-1; positive=cfg.grfK+1:cfg.grfN-1;
 if cfg.grfJ<cfg.grfK, same=negative; opposite=positive;
 else, same=positive; opposite=negative; end
 if ~isempty(opposite), values(opposite)=min(.95,sum(values(same))/numel(opposite)); end
 ratioNodes=setdiff(2:cfg.grfN-1,[cfg.grfK cfg.grfJ]);
 seed(7:6+cfg.grfN-4)=values(ratioNodes);
 starts(indices(1),:)=(seed-lo)./width;
 problem.x0=starts(indices(1),:);
 fprintf('Horizontal GRF: k=%d, j=%d, %d broad starts\n',...
  cfg.grfK,cfg.grfJ,numel(indices));
 [xx,ff,~,oo,ss]=run(ms,problem,CustomStartPointSet(starts(indices,:)));
 collectMessages();
 distinctGlobalSolutions=distinctGlobalSolutions+numel(ss);
 og.localSolverTotal=og.localSolverTotal+oo.localSolverTotal;
 og.localSolverSuccess=og.localSolverSuccess+oo.localSolverSuccess;
 if isempty(xx), continue; end
 trial=evaluate(xx); violation=trial.maxViolation;
 feasible=violation<=cfg.constraintTolerance;
 incumbentFeasible=bestViolation<=cfg.constraintTolerance;
 if isempty(xg) || (feasible && ~incumbentFeasible) || ...
   (feasible && incumbentFeasible && ff<fg) || ...
   (~feasible && ~incumbentFeasible && violation<bestViolation)
  xg=xx; fg=ff; selectedPair=pairs(group,:); bestViolation=violation;
 end
end
cfg.grfK=selectedPair(1); cfg.grfJ=selectedPair(2); lastX=[]; lastR=[];
globalElapsed=toc(stageClock);
if isempty(xg), error('EFLM:NoSolution','No global-stage solution found.'); end
stage=2; stageClock=tic;
localLo=max(0,xg-cfg.localRadius); localHi=min(1,xg+cfg.localRadius);
problem.x0=xg; problem.lb=localLo; problem.ub=localHi;
startsLocal=[xg;localLo+(localHi-localLo).*rand(cfg.localStarts-1,nvars)];
[xl,fl,~,ol,sl]=run(ms,problem,CustomStartPointSet(startsLocal));
collectMessages();
localElapsed=toc(stageClock);
best=xg;
if ~isempty(xl)
 rg=evaluate(xg); rl=evaluate(xl);
 if (rl.maxViolation<=cfg.constraintTolerance && (rg.maxViolation>cfg.constraintTolerance || fl<fg)) || ...
   (rg.maxViolation>cfg.constraintTolerance && rl.maxViolation<rg.maxViolation)
  best=xl;
 end
end
% Inspect the refined time mesh before reporting success. If necessary,
% continue constrained refinement on that same dense mesh.
dense=simulate(lo+best.*width,p,tab,cfg,cfg.finalNodes);
stage=3; stageClock=tic; cfg.nodes=cfg.finalNodes; lastX=[];
active=[];
polishStats=struct('starts',0,'successfulStarts',0,'iterations',0,...
 'functionEvaluations',0,'elapsed',0);
% Keep a verified feasible incumbent; never replace it by a worse violation.
% Slowing a candidate can increase vertical sag. Use the initial posture as
% a fallback only when it also succeeds on the final high-density mesh.
safe=(cfg.initial-lo)./width;
safeDense=[];
try
 safeDense=simulate(cfg.initial,p,tab,cfg,cfg.finalNodes);
catch ME
 if ~startsWith(ME.identifier,'EFLM:'), rethrow(ME); end
 % The user-supplied initial point is an optimizer seed, not a guaranteed
 % high-density feasible fallback. Keep the optimized dense candidate.
end
incumbent=best; incumbentDense=dense;
if ~isempty(safeDense) && safeDense.maxViolation<=cfg.constraintTolerance
 incumbent=safe; incumbentDense=safeDense;
end
for pass=1:16
 % Constraint generation: selected time rows are smooth, unlike a time maximum.
 for channel=1:size(dense.constraintChannels,2)
  [~,idx]=maxk(dense.constraintChannels(:,channel),3);
  active=[active;(channel-1)*cfg.nodes+idx]; %#ok<AGROW>
 end
 active=unique([active;(numel(dense.constraintChannels)+1:numel(dense.constraints))']);
 [candidate,~,flag,out]=fmincon(@objective,best,[],[],[],[],localLo,localHi,...
  @activeConstraints,options);
 check=simulate(lo+candidate.*width,p,tab,cfg,cfg.finalNodes);
 polishStats.starts=polishStats.starts+1;
 polishStats.successfulStarts=polishStats.successfulStarts+(flag>0);
 if isfield(out,'iterations'), polishStats.iterations=polishStats.iterations+out.iterations; end
 if isfield(out,'funcCount'), polishStats.functionEvaluations=polishStats.functionEvaluations+out.funcCount; end
 if check.maxViolation<=cfg.constraintTolerance && ...
   (incumbentDense.maxViolation>cfg.constraintTolerance || check.objective<incumbentDense.objective)
  incumbent=candidate; incumbentDense=check;
 end
 best=candidate;
 dense=check;
 if check.maxViolation<=cfg.constraintTolerance, break; end
end
best=incumbent;
globalStats=multiStartStats(og,cfg.starts,globalElapsed,...
 stageIterations(1),stageFunctionEvaluations(1),distinctGlobalSolutions);
localStats=multiStartStats(ol,cfg.localStarts,localElapsed,...
 stageIterations(2),stageFunctionEvaluations(2),numel(sl));
polishStats.elapsed=toc(stageClock);
result=struct('parameters',p,'settings',cfg,...
 'stageStats',struct('global',globalStats,'local',localStats,'polish',polishStats),...
 'bestVariables',lo+best.*width,'elapsed',toc(started));
result.final=simulate(result.bestVariables,p,tab,cfg,cfg.finalNodes);
result.valid=result.final.maxViolation<=cfg.constraintTolerance && ...
 max(abs(result.final.body.periodic([1 3])))<1e-8 && result.final.body.distance>0;
% Horizontal periodicity is the analytic impulse equality included in maxViolation.
result.status='completed_but_infeasible';
if result.valid, result.status='feasible_within_sampled_constraints'; end
result.flightVisualization='Ballistic body only; leg hidden in flight';
result.globalOptimality='MultiStart does not certify global optimality; finite search box';
result.boundaryDistance=min(best,1-best);
result.matlabVersion=version; result.createdAt=char(datetime('now','TimeZone','Asia/Tokyo'));
 function r=evaluate(x)
  if isequal(x,lastX), r=lastR; return; end
  a=lo+x.*width;
  try
   r=simulate(a,p,tab,cfg,cfg.nodes);
   if r.body.distance<=0 || ~isfinite(r.objective), error('EFLM:Distance','Nonpositive travel'); end
  catch ME
   if ~startsWith(ME.identifier,'EFLM:'), rethrow(ME); end
   r=struct('objective',cfg.penalty+sum(x.^2),'constraints',ones(16*cfg.nodes+cfg.shapeConstraintCount+3,1),...
    'constraintEnvelope',ones(19+cfg.shapeConstraintCount,1),'equalityConstraints',1,...
    'maxViolation',1,'cot',NaN,'body',struct('speed',NaN),'success',false);
  end
  lastX=x; lastR=r;
 end
 function f=objective(x), r=evaluate(x); f=r.objective; end
 function [c,ceq]=constraints(x), r=evaluate(x); c=r.constraintEnvelope; ceq=r.equalityConstraints; end
 function [c,ceq]=activeConstraints(x), r=evaluate(x); c=r.constraints(active); ceq=r.equalityConstraints; end
 function stop=recordSolverStats(~,optimValues,state)
  if strcmp(state,'done') && stage>=1 && stage<=3
   count=0;
   if isfield(optimValues,'funccount'), count=optimValues.funccount; end
   if cfg.useParallel && ~isempty(getCurrentTask())
    send(messages,struct('kind','stats','stage',stage,...
     'iterations',optimValues.iteration,'evaluations',count));
   else
    stageIterations(stage)=stageIterations(stage)+optimValues.iteration;
    stageFunctionEvaluations(stage)=stageFunctionEvaluations(stage)+count;
   end
  end
  stop=false;
 end
 function collectMessages()
  if ~cfg.useParallel, return; end
  while true
   msg=poll(messages,.1);
   if isempty(msg), break; end
   stageIterations(msg.stage)=stageIterations(msg.stage)+msg.iterations;
   stageFunctionEvaluations(msg.stage)=stageFunctionEvaluations(msg.stage)+msg.evaluations;
  end
 end
end

function stats = multiStartStats(output, starts, elapsed, iterations, evaluations, distinctSolutions)
successful=distinctSolutions;
if isfield(output,'localSolverSuccess'), successful=output.localSolverSuccess; end
actualStarts=starts;
if isfield(output,'localSolverTotal'), actualStarts=output.localSolverTotal; end
stats=struct('starts',actualStarts,'successfulStarts',successful,...
 'iterations',iterations,'functionEvaluations',evaluations,'elapsed',elapsed,...
 'distinctSolutions',distinctSolutions);
end
