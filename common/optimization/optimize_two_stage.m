function result = optimize_two_stage(p,tab,cfg,folder)
% MultiStart global exploration followed by a smaller-box MultiStart refinement.
rng(cfg.seed,'twister'); initialRng=rng;
history=zeros(0,10); failures=cell(0,2); lastX=[]; lastR=[];
stage=0; started=tic; stageClock=tic;
lo=cfg.lower; hi=cfg.upper; width=hi-lo;
options=optimoptions('fmincon','Algorithm','sqp','Display','off',...
 'MaxIterations',cfg.maxIterations,'MaxFunctionEvaluations',cfg.maxEvaluations,...
 'ConstraintTolerance',cfg.constraintTolerance,'StepTolerance',cfg.stepTolerance,...
 'FiniteDifferenceStepSize',1e-4,'OutputFcn',@stopSlow);
ms=MultiStart('Display','iter','UseParallel',false,'StartPointsToRun','all',...
 'MaxTime',cfg.stageSeconds);
stage=1; stageClock=tic;
x0=(cfg.initial-lo)./width;
starts=[x0;rand(cfg.starts-1,4)];
problem=createOptimProblem('fmincon','x0',x0,'objective',@objective,...
 'nonlcon',@constraints,'lb',zeros(1,4),'ub',ones(1,4),'options',options);
[xg,fg,eg,og,sg]=run(ms,problem,CustomStartPointSet(starts));
save(fullfile(folder,'global_checkpoint.mat'),'xg','fg','eg','og','sg','history','failures','cfg','p');
stage=2; stageClock=tic;
localLo=max(0,xg-cfg.localRadius); localHi=min(1,xg+cfg.localRadius);
problem.x0=xg; problem.lb=localLo; problem.ub=localHi;
startsLocal=[xg;localLo+(localHi-localLo).*rand(cfg.localStarts-1,4)];
[xl,fl,el,ol,sl]=run(ms,problem,CustomStartPointSet(startsLocal));
if fl<fg, best=xl; else, best=xg; end
result=struct('parameters',p,'settings',cfg,'geometryTable',tab,...
 'initialRng',initialRng,'finalRng',rng,'globalStartPoints',lo+starts.*width,...
 'localStartPoints',lo+startsLocal.*width,'globalBounds',[lo;hi],...
 'localBounds',[lo+localLo.*width;lo+localHi.*width],...
 'global',struct('x',lo+xg.*width,'f',fg,'exitflag',eg,'output',og,'solutions',sg),...
 'local',struct('x',lo+xl.*width,'f',fl,'exitflag',el,'output',ol,'solutions',sl),...
 'bestVariables',lo+best.*width,'history',history,'failures',{failures},'elapsed',toc(started));
result.final=sim_cycle_elastic_approx(result.bestVariables,p,tab,cfg,cfg.finalNodes);
result.valid=result.final.maxViolation<=cfg.constraintTolerance;
result.boundaryDistance=min(best,1-best);
result.matlabVersion=version; result.createdAt=char(datetime('now','TimeZone','Asia/Tokyo'));
save(fullfile(folder,'result.mat'),'result','-v7.3');
writematrix(history,fullfile(folder,'optimization_history.csv'));

 function r=evaluate(x)
  if isequal(x,lastX), r=lastR; return; end
  a=lo+x.*width;
  try
   r=sim_cycle_elastic_approx(a,p,tab,cfg,cfg.nodes);
   if r.body.distance<=0 || ~isfinite(r.objective), error('EFLM:Distance','Nonpositive travel'); end
  catch ME
   if ~startsWith(ME.identifier,'EFLM:'), rethrow(ME); end
   r=struct('objective',cfg.penalty+sum(x.^2),'constraints',ones(13*cfg.nodes+1,1),...
    'maxViolation',1,'cot',NaN,'body',struct('speed',NaN),'success',false);
   failures(end+1,:)={a,ME.message};
  end
  history(end+1,:)=[stage,toc(started),a,r.objective,r.body.speed,r.cot,r.maxViolation];
  lastX=x; lastR=r;
 end
 function f=objective(x), r=evaluate(x); f=r.objective; end
 function [c,ceq]=constraints(x), r=evaluate(x); c=r.constraints; ceq=[]; end
 function stop=stopSlow(~,~,~), stop=toc(stageClock)>cfg.stageSeconds; end
end
