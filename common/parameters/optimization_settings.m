function cfg = optimization_settings()
% Initial finite search box, not a claim of an unrestricted global optimum.
% vx0 [m/s], thetaL [rad], r0 [m], stanceScale [-], duty [-], Ix [N s].
cfg.layoutVersion=2;
cfg.lower=[1 deg2rad(40) .26 .5 .3 .1];
cfg.upper=[3.5 deg2rad(65) .33 2 .6 4];
cfg.initial=[2.2 deg2rad(50) .30 1 .5 1.13];
cfg.seed=20261103; cfg.startsPerK=2000; cfg.localStarts=5;
cfg.useParallel=true; cfg.parallelWorkers=4;
cfg.localRadius=.15; cfg.nodes=121; cfg.finalNodes=1201;
% R2026b Update 1 in this environment lacks the nlpSQP implementation
% required by Algorithm='sqp'. The legacy SQP implementation is available.
cfg.algorithm='sqp-legacy';
cfg.w1=10; cfg.w2=1;
cfg.targetSpeed=4; cfg.targetCoT=.3;
cfg.maxIterations=120; cfg.maxEvaluations=1400;
cfg.constraintTolerance=1e-5; cfg.stepTolerance=1e-7;
cfg.evaluationSeconds=5;
cfg.penalty=1e5; cfg.video=true; cfg.fps=30; cfg.cycles=3; cfg.playbackSpeed=.1;
cfg.minimumVideoSeconds=4;
cfg.minimumStance=.2; cfg.minimumFlight=.2;
cfg.grfXN=15; % Independent horizontal-force control point count.
cfg.grfZN=9; % Independent vertical-force control point count.
cfg.grfZBetaStart=[0 .5 0];
cfg.grfZBetaEnd=[-.5 0 0];
cfg.restrictGrfK=false;
cfg.grfKSelection=[];
cfg.grfXBetaStart=[-.5 0 0];
cfg.grfXBetaEnd=[-.5 0 0];
end
