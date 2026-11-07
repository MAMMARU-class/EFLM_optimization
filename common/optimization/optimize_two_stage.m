function result = optimize_two_stage(p,tab,cfg,folder)
% MultiStart global exploration followed by a smaller-box MultiStart refinement.
rng(cfg.seed,'twister'); initialRng=rng;
prepare_optimization_pool(cfg);
simulate=str2func(cfg.simulationFunction);
messages=[];
if cfg.useParallel, messages=parallel.pool.PollableDataQueue; end
history=zeros(0,11); failures=cell(0,2); lastX=[]; lastR=[];
stageIterations=zeros(1,3); stageFunctionEvaluations=zeros(1,3);
stage=0; started=tic; stageClock=tic; runWall=now; stageWall=now;
lo=cfg.lower; hi=cfg.upper; width=hi-lo;
options=optimoptions('fmincon','Algorithm',cfg.algorithm,'Display','off',...
 'MaxIterations',cfg.maxIterations,'MaxFunctionEvaluations',cfg.maxEvaluations,...
 'ConstraintTolerance',cfg.constraintTolerance,'StepTolerance',cfg.stepTolerance,...
 'FiniteDifferenceStepSize',1e-6,'OutputFcn',@stopSlow);
ms=MultiStart('Display','iter','UseParallel',cfg.useParallel,'StartPointsToRun','all',...
 'MaxTime',cfg.stageSeconds);
stage=1; stageClock=tic; stageWall=now;
x0=(cfg.initial-lo)./width;
starts=[x0;rand(cfg.starts-1,5)];
% Half of the starts sample plausible postures, retaining broad-box starts.
% This is random MultiStart seeding, never a parameter grid search.
count=floor((cfg.starts-1)/2);
if count>0
 angles=deg2rad(1+8*rand(count,1));
 ts=cfg.minimumStance+.02*rand(count,1); tf=cfg.minimumFlight+.02*rand(count,1);
 lengths=lo(5)+(hi(5)-lo(5))*rand(count,1);
 amplitudes=min(hi(1),max(lo(1),rand(count,1)));
 velocities=2*lengths.*sin(angles)./ts;
 if strcmp(cfg.method,'mean_velocity')
  velocities=velocities+amplitudes.*ts/(2*pi*p.m);
 end
 seeds=[amplitudes,angles,velocities,ts./(ts+tf),lengths];
 starts(2:count+1,:)=min(1,max(0,(seeds-lo)./width));
end
problem=createOptimProblem('fmincon','x0',x0,'objective',@objective,...
 'nonlcon',@constraints,'lb',zeros(1,5),'ub',ones(1,5),'options',options);
[xg,fg,eg,og,sg]=run(ms,problem,CustomStartPointSet(starts));
collectMessages();
globalElapsed=toc(stageClock);
save(fullfile(folder,'global_checkpoint.mat'),'xg','fg','eg','og','sg','history','failures','cfg','p');
if isempty(xg), error('EFLM:NoSolution','No global-stage solution; inspect global_checkpoint.mat'); end
stage=2; stageClock=tic; stageWall=now;
localLo=max(0,xg-cfg.localRadius); localHi=min(1,xg+cfg.localRadius);
problem.x0=xg; problem.lb=localLo; problem.ub=localHi;
startsLocal=[xg;localLo+(localHi-localLo).*rand(cfg.localStarts-1,5)];
[xl,fl,el,ol,sl]=run(ms,problem,CustomStartPointSet(startsLocal));
collectMessages();
localElapsed=toc(stageClock);
if ~isempty(xl) && fl<fg, best=xl; else, best=xg; end
% Inspect the refined time mesh before reporting success. If necessary,
% continue constrained refinement on that same dense mesh.
denseFallback='';
try
 dense=simulate(lo+best.*width,p,tab,cfg,cfg.finalNodes);
catch ME
 if ~isfield(cfg,'model') || ~strcmp(cfg.model,'EFLM') || ~startsWith(ME.identifier,'EFLM:'), rethrow(ME); end
 % The 5-coordinate assembly can lose its branch between coarse time nodes.
 % Recover only from a genuinely feasible dense candidate, never a penalty.
 good=history(history(:,11)<=cfg.constraintTolerance,:);
 [~,order]=sort(good(:,8)); good=good(order,:);
 candidates=[good(1:min(20,size(good,1)),3:7);cfg.initial]; recovered=false;
 for candidateIndex=1:size(candidates,1)
  try
   trialDense=simulate(candidates(candidateIndex,:),p,tab,cfg,cfg.finalNodes);
   if trialDense.maxViolation<=cfg.constraintTolerance
    best=(candidates(candidateIndex,:)-lo)./width; dense=trialDense; recovered=true; break;
   end
  catch candidateError
   if ~startsWith(candidateError.identifier,'EFLM:'), rethrow(candidateError); end
  end
 end
 if ~recovered, rethrow(ME); end
 denseFallback=ME.message;
 localLo=max(0,best-cfg.localRadius); localHi=min(1,best+cfg.localRadius);
end
polish=struct('required',true,'initialViolation',dense.maxViolation);
polish.denseFallback=denseFallback;
stage=3; stageClock=tic; stageWall=now; cfg.nodes=cfg.finalNodes; lastX=[];
active=[]; polish.passes=cell(0,1);
% Keep a verified feasible incumbent; never replace it by a worse violation.
% Slowing a candidate can increase vertical sag. Use the initial posture as
% a fallback only when it also succeeds on the final high-density mesh.
safe=(cfg.initial-lo)./width;
safeDense=[];
polish.feasibleBaseline=struct('x',cfg.initial,'available',false,...
 'maxViolation',Inf,'failure','');
try
 safeDense=simulate(cfg.initial,p,tab,cfg,cfg.finalNodes);
 polish.feasibleBaseline.available=true;
 polish.feasibleBaseline.maxViolation=safeDense.maxViolation;
catch ME
 if ~startsWith(ME.identifier,'EFLM:'), rethrow(ME); end
 % The user-supplied initial point is an optimizer seed, not a guaranteed
 % high-density feasible fallback. Keep the optimized dense candidate.
 polish.feasibleBaseline.failure=sprintf('%s: %s',ME.identifier,ME.message);
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
 active=unique([active;(numel(dense.constraints)-2:numel(dense.constraints))']);
 [candidate,fval,flag,out]=fmincon(@objective,best,[],[],[],[],localLo,localHi,...
  @activeConstraints,options);
 check=simulate(lo+candidate.*width,p,tab,cfg,cfg.finalNodes);
 polish.passes{pass}=struct('x',lo+candidate.*width,'f',fval,'exitflag',flag,...
  'output',out,'maxViolation',check.maxViolation,'activeIndices',active);
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
 stageIterations(1),stageFunctionEvaluations(1),numel(sg));
localStats=multiStartStats(ol,cfg.localStarts,localElapsed,...
 stageIterations(2),stageFunctionEvaluations(2),numel(sl));
polishStats=polishPassStats(polish.passes,toc(stageClock));
result=struct('parameters',p,'settings',cfg,'geometryTable',tab,...
 'initialRng',initialRng,'finalRng',rng,'globalStartPoints',lo+starts.*width,...
 'localStartPoints',lo+startsLocal.*width,'globalBounds',[lo;hi],...
 'localBounds',[lo+localLo.*width;lo+localHi.*width],...
 'global',struct('x',lo+xg.*width,'f',fg,'exitflag',eg,'output',og,'solutions',sg),...
 'local',struct('x',lo+xl.*width,'f',fl,'exitflag',el,'output',ol,'solutions',sl),...
 'polish',polish,...
 'stageStats',struct('global',globalStats,'local',localStats,'polish',polishStats),...
 'bestVariables',lo+best.*width,'history',history,'failures',{failures},'elapsed',toc(started));
result.final=simulate(result.bestVariables,p,tab,cfg,cfg.finalNodes);
result.valid=result.final.maxViolation<=cfg.constraintTolerance && ...
 max(abs(result.final.body.periodic))<1e-8 && result.final.body.distance>0;
result.status='completed_but_infeasible';
if result.valid, result.status='feasible_within_sampled_constraints'; end
result.flightVisualization='Ballistic body only; leg hidden in flight';
result.globalOptimality='MultiStart does not certify global optimality; finite search box';
result.boundaryDistance=min(best,1-best);
result.matlabVersion=version; result.createdAt=char(datetime('now','TimeZone','Asia/Tokyo'));
save(fullfile(folder,'result.mat'),'result','-v7.3');
writetable(array2table(history,'VariableNames',{'stage','seconds','AmpH','thetaL','v','Duty','r0',...
 'objective','speed','CoT','maxViolation'}),fullfile(folder,'optimization_history.csv'));

 function r=evaluate(x)
  if isequal(x,lastX), r=lastR; return; end
  a=lo+x.*width;
  failure='';
  try
   r=simulate(a,p,tab,cfg,cfg.nodes);
   if r.body.distance<=0 || ~isfinite(r.objective), error('EFLM:Distance','Nonpositive travel'); end
  catch ME
   if ~startsWith(ME.identifier,'EFLM:'), rethrow(ME); end
   r=struct('objective',cfg.penalty+sum(x.^2),'constraints',ones(13*cfg.nodes+3,1),...
    'constraintEnvelope',ones(16,1),...
    'maxViolation',1,'cot',NaN,'body',struct('speed',NaN),'success',false);
   failure=ME.message;
  end
  row=[stage,(now-runWall)*86400,a,r.objective,r.body.speed,r.cot,r.maxViolation];
  if cfg.useParallel && ~isempty(getCurrentTask())
   send(messages,struct('kind','evaluation','row',row,'failure',failure));
  else
   history(end+1,:)=row;
   if ~isempty(failure), failures(end+1,:)={a,failure}; end
  end
  lastX=x; lastR=r;
 end
 function f=objective(x), r=evaluate(x); f=r.objective; end
 function [c,ceq]=constraints(x), r=evaluate(x); c=r.constraintEnvelope; ceq=[]; end
 function [c,ceq]=activeConstraints(x), r=evaluate(x); c=r.constraints(active); ceq=[]; end
 function stop=stopSlow(~,optimValues,state)
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
  stop=(now-stageWall)*86400>cfg.stageSeconds;
 end
 function collectMessages()
  if ~cfg.useParallel, return; end
  while true
   msg=poll(messages,1);
   if isempty(msg), break; end
   if strcmp(msg.kind,'evaluation')
    history(end+1,:)=msg.row;
    if ~isempty(msg.failure), failures(end+1,:)={msg.row(3:7),msg.failure}; end
   else
    stageIterations(msg.stage)=stageIterations(msg.stage)+msg.iterations;
    stageFunctionEvaluations(msg.stage)=stageFunctionEvaluations(msg.stage)+msg.evaluations;
   end
  end
  history=sortrows(history,[1 2]);
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

function stats = polishPassStats(passes, elapsed)
stats=struct('starts',numel(passes),'successfulStarts',0,'iterations',0,...
 'functionEvaluations',0,'elapsed',elapsed);
for i=1:numel(passes)
 pass=passes{i};
 if pass.exitflag>0, stats.successfulStarts=stats.successfulStarts+1; end
 if isfield(pass.output,'iterations'), stats.iterations=stats.iterations+pass.output.iterations; end
 if isfield(pass.output,'funcCount'), stats.functionEvaluations=stats.functionEvaluations+pass.output.funcCount; end
end
end
