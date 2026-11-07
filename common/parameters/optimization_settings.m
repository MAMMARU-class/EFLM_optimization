function cfg = optimization_settings()
% Initial finite search box, not a claim of an unrestricted global optimum.
% Variables: AmpH [N], thetaL [rad], v [m/s], Duty, r0 [m], stanceScale.
p=robot_parameters(); % p.leg supplies only the default initial r0.
cfg.lower=[0 deg2rad(.5) .03 .10 .25 .5];
cfg.upper=[50 deg2rad(35) 2 .90 .34 2];
cfg.initial=[.5 deg2rad(8) 2*p.leg*sin(deg2rad(8))/.2 .5 p.leg 1];
cfg.seed=20261103; cfg.starts=16; cfg.localStarts=5;
cfg.useParallel=true; cfg.parallelWorkers=4;
cfg.localRadius=.15; cfg.nodes=121; cfg.finalNodes=1201;
% R2026b Update 1 in this environment lacks the nlpSQP implementation
% required by Algorithm='sqp'. The legacy SQP implementation is available.
cfg.algorithm='sqp-legacy';
cfg.w1=10; cfg.w2=1;
cfg.targetSpeed=4; cfg.targetCoT=.3;
cfg.maxIterations=120; cfg.maxEvaluations=1400;
cfg.constraintTolerance=1e-5; cfg.stepTolerance=1e-7;
cfg.stageSeconds=12*60*60; cfg.evaluationSeconds=5;
cfg.penalty=1e5; cfg.video=true; cfg.fps=30; cfg.cycles=3; cfg.playbackSpeed=.1;
cfg.minimumVideoSeconds=4;
cfg.minimumStance=.2; cfg.minimumFlight=.2;
end
