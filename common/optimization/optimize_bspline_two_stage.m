function result=optimize_bspline_two_stage(p,tab,cfg,folder)
% Broad/narrow MultiStart in resumable chunks; dense polish remains unchanged.
lo=cfg.lower; width=cfg.upper-lo; nv=numel(lo); timer=tic;
lastX=[]; lastR=[]; stage=0; completed=0; feasible=0;
batchCompleted=0; batchFeasible=0; lineLength=0;
progressGuard=onCleanup(@finishProgress); %#ok<NASGU>
if cfg.useParallel
 queue=parallel.pool.DataQueue; afterEach(queue,@receiveStart);
else
 queue=[];
end
op=bspline_operators(cfg.bsplineXN,1);
A=zeros(2,nv); A(1,cfg.index.fx)=op.I(2:end-1)/cfg.bsplineForceBound;
A(2,cfg.index.fz)=1; Aeq=A.*width; beq=[0;1]-A*lo';
options=optimoptions('fmincon','Algorithm',cfg.algorithm,'Display','off',...
 'MaxIterations',cfg.maxIterations,'MaxFunctionEvaluations',cfg.maxEvaluations,...
 'ConstraintTolerance',cfg.constraintTolerance,'StepTolerance',cfg.stepTolerance,...
 'FiniteDifferenceStepSize',1e-6,'OutputFcn',@recordStart);
ms=MultiStart('Display','off','UseParallel',cfg.useParallel,'StartPointsToRun','all','MaxTime',Inf);
checkpoints=fullfile(folder,'checkpoints');
if ~isfolder(checkpoints), mkdir(checkpoints); end
globalStarts=fullfile(checkpoints,'broad_starts.mat');
if isfile(globalStarts)
 saved=load(globalStarts,'starts','state'); starts=saved.starts; rng(saved.state);
else
 rng(cfg.seed,'twister'); physical=lo+rand(cfg.broadStarts,nv).*width;
 w=op.I(2:end-1); c=physical(:,cfg.index.fx);
 c=c-(c*w')/(w*w')*w;
 c=c./max(1,max(abs(c),[],2)/cfg.bsplineForceBound);
 physical(:,cfg.index.fx)=c;
 z=physical(:,cfg.index.fz); physical(:,cfg.index.fz)=z./sum(z,2);
 physical(1,:)=cfg.initial; starts=(physical-lo)./width;
 state=rng; atomicSave(globalStarts,struct('starts',starts,'state',state));
end
if size(starts,1)~=cfg.broadStarts || size(starts,2)~=nv
 error('EFLM:ResumeSettings','Broad start points do not match saved settings.');
end
problem=createOptimProblem('fmincon','x0',starts(1,:),'objective',@objective,...
 'nonlcon',@constraints,'Aeq',Aeq,'beq',beq,'lb',zeros(1,nv),'ub',ones(1,nv),'options',options);
[xg,globalStats]=searchChunks('broad',starts,problem);
if isempty(xg), error('EFLM:NoSolution','No broad-stage B-spline solution found.'); end
localLo=max(0,xg-cfg.localRadius); localHi=min(1,xg+cfg.localRadius);
problem.x0=xg; problem.lb=localLo; problem.ub=localHi;
localStartsPath=fullfile(checkpoints,'narrow_starts.mat');
if isfile(localStartsPath)
 saved=load(localStartsPath,'starts','broadWinner');
 if ~isequal(saved.broadWinner,xg), error('EFLM:ResumeSettings','Narrow starts belong to a different broad winner.'); end
 local=saved.starts;
else
 rng(cfg.seed+1,'twister'); local=[xg;localLo+rand(cfg.localStarts-1,nv).*(localHi-localLo)];
 atomicSave(localStartsPath,struct('starts',local,'broadWinner',xg));
end
if size(local,1)~=cfg.localStarts || size(local,2)~=nv
 error('EFLM:ResumeSettings','Narrow start points do not match saved settings.');
end
[xl,localStats]=searchChunks('narrow',local,problem);
best=xg;
if ~isempty(xl) && better(evaluate(xl),evaluate(xg)), best=xl; end
stage=3; cfg.nodes=cfg.finalNodes; lastX=[]; lastR=[]; stageClock=tic;
incumbent=best; dense=evaluate(best); incumbentDense=dense; active=[];
safe=(cfg.initial-lo)./width; safeDense=evaluate(safe);
if safeDense.success && safeDense.maxViolation<=cfg.constraintTolerance && better(safeDense,incumbentDense)
 incumbent=safe; incumbentDense=safeDense;
end
if ~dense.success && incumbentDense.success, best=incumbent; dense=incumbentDense; end
polish=struct('starts',0,'successfulStarts',0,'iterations',0,'functionEvaluations',0,'elapsed',0);
for pass=1:16
 if ~dense.success, break; end
 for ch=1:size(dense.constraintChannels,2)
  [~,rows]=maxk(dense.constraintChannels(:,ch),3);
  active=[active;(ch-1)*cfg.nodes+rows]; %#ok<AGROW>
 end
 active=unique([active;(numel(dense.constraintChannels)+1:numel(dense.constraints))']);
 [candidate,~,flag,out]=fmincon(@objective,best,[],[],Aeq,beq,...
  localLo,localHi,@activeConstraints,options);
 check=evaluate(candidate); polish.starts=pass;
 polish.successfulStarts=polish.successfulStarts+(flag>0);
 polish.iterations=polish.iterations+out.iterations;
 polish.functionEvaluations=polish.functionEvaluations+out.funcCount;
 if better(check,incumbentDense), incumbent=candidate; incumbentDense=check; end
 best=candidate; dense=check;
 if check.success && check.maxViolation<=cfg.constraintTolerance, break; end
end
polish.elapsed=toc(stageClock);
if ~incumbentDense.success
 error('EFLM:NoSolution','Dense evaluation failed: %s (%s)',...
  incumbentDense.errorMessage,incumbentDense.errorIdentifier);
end
result=struct('parameters',p,'settings',cfg,'bestVariables',lo+incumbent.*width,...
 'final',incumbentDense,'elapsed',toc(timer));
result.valid=result.final.maxViolation<=cfg.constraintTolerance && result.final.body.distance>0 && ...
 max(abs(result.final.body.periodic([1 3])))<1e-8;
result.status='completed_but_infeasible';
if result.valid, result.status='feasible_within_sampled_constraints'; end
result.stageStats=struct('global',globalStats,'local',localStats,'polish',polish);

 function [winner,stats]=searchChunks(label,points,baseProblem)
  stage=1; if strcmp(label,'narrow'), stage=2; end
  total=size(points,1); completed=0; feasible=0; batchCompleted=0; batchFeasible=0;
  winner=[]; winnerResult=[];
  elapsed=0; solverTotal=0; solverSuccess=0; solutions=0;
  chunkSize=cfg.save_interval; chunks=ceil(total/chunkSize);
  fprintf('%s: %d starts, %d nodes\n',label,total,cfg.nodes);
  for chunk=1:chunks
   first=(chunk-1)*chunkSize+1; last=min(chunk*chunkSize,total);
   path=fullfile(checkpoints,sprintf('%s_%05d.mat',label,chunk));
   if isfile(path)
    saved=load(path,'record'); record=saved.record;
    if record.first~=first || record.last~=last || record.total~=total
     error('EFLM:ResumeSettings','Checkpoint %s does not match start positions.',path);
    end
   else
    chunkClock=tic; batchCompleted=0; batchFeasible=0;
    baseProblem.x0=points(first,:);
    [x,~,~,out,ss]=run(ms,baseProblem,CustomStartPointSet(points(first:last,:)));
    if cfg.useParallel, drawnow; end % Deliver completed worker messages before saving.
    record=struct('first',first,'last',last,'total',total,'x',x,...
     'elapsed',toc(chunkClock),'localSolverTotal',out.localSolverTotal,...
     'localSolverSuccess',out.localSolverSuccess,'solutions',numel(ss),...
     'reportedStarts',batchCompleted,'feasibleStarts',batchFeasible);
    atomicSave(path,struct('record',record));
   end
   if ~isempty(record.x)
    r=evaluate(record.x);
    if isempty(winner) || better(r,winnerResult), winner=record.x; winnerResult=r; end
   end
   completed=last; feasible=feasible+record.feasibleStarts;
   batchCompleted=0; batchFeasible=0;
   elapsed=elapsed+record.elapsed; solverTotal=solverTotal+record.localSolverTotal;
   solverSuccess=solverSuccess+record.localSolverSuccess; solutions=solutions+record.solutions;
   showProgress(label,total);
  end
  fprintf('\n'); lineLength=0;
  stats=struct('starts',solverTotal,'successfulStarts',solverSuccess,...
   'distinctSolutions',solutions,'feasibleStarts',feasible,'elapsed',elapsed);
 end
 function stop=recordStart(x,~,state)
  stop=false;
  if ~strcmp(state,'done') || stage<1 || stage>2, return; end
  r=evaluate(x); isFeasible=r.success && r.maxViolation<=cfg.constraintTolerance;
  if cfg.useParallel && ~isempty(getCurrentTask())
   send(queue,double(isFeasible));
  else
   receiveStart(double(isFeasible));
  end
 end
 function receiveStart(isFeasible)
  batchCompleted=batchCompleted+1; batchFeasible=batchFeasible+isFeasible;
  if mod(completed+batchCompleted,max(1,ceil(cfg.save_interval/20)))==0
   label='broad'; total=cfg.broadStarts;
   if stage==2, label='narrow'; total=cfg.localStarts; end
   showProgress(label,total);
  end
 end
 function showProgress(label,total)
  done=completed+batchCompleted; good=feasible+batchFeasible;
  fraction=min(1,done/total); ratio=good/max(1,done); width=20;
  a=floor(width*fraction); b=floor(width*ratio);
  line=sprintf('%s progress [%s%s] %3.0f%% | feasible [%s%s] %3.0f%% (%d/%d)',...
   label,repmat('=',1,a),repmat(' ',1,width-a),100*fraction,...
   repmat('=',1,b),repmat(' ',1,width-b),100*ratio,good,done);
  if lineLength>0, fprintf(1,'%s',repmat(char(8),1,lineLength)); end
  padding=max(0,lineLength-numel(line)); fprintf(1,'%s%s',line,repmat(' ',1,padding));
  lineLength=numel(line)+padding;
 end
 function finishProgress()
  if lineLength>0, fprintf('\n'); lineLength=0; end
 end
 function yes=better(a,b)
  yes=a.success && (~b.success || ...
   (a.maxViolation<=cfg.constraintTolerance && ...
    (b.maxViolation>cfg.constraintTolerance || a.objective<b.objective)) || ...
   (b.maxViolation>cfg.constraintTolerance && a.maxViolation<b.maxViolation));
 end
 function r=evaluate(x)
  if isequal(x,lastX), r=lastR; return; end
  try
   r=sim_cycle_stage1_point_mass(lo+x.*width,p,tab,cfg,cfg.nodes);
   if r.body.distance<=0 || ~isfinite(r.objective), error('EFLM:Distance','Invalid displacement.'); end
  catch ME
   if ~startsWith(ME.identifier,'EFLM:'), rethrow(ME); end
   r=struct('success',false,'objective',cfg.penalty+sum(x.^2),'maxViolation',Inf,...
    'constraints',ones(16*cfg.nodes+4,1),'constraintEnvelope',ones(20,1),...
    'errorIdentifier',ME.identifier,'errorMessage',ME.message);
  end
  lastX=x; lastR=r;
 end
 function f=objective(x), r=evaluate(x); f=r.objective; end
 function [c,e]=constraints(x), r=evaluate(x); c=r.constraintEnvelope; e=[]; end
 function [c,e]=activeConstraints(x), r=evaluate(x); c=r.constraints(active); e=[]; end
 function atomicSave(path,value)
  temporary=[path '.tmp']; save(temporary,'-struct','value','-v7');
  movefile(temporary,path,'f');
 end
end
