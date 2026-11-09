function folder = smoke_hermite_optimization()
% Small integration run: exercise categorical MultiStart, equality constraints,
% dense refinement, and final-result/CSV/report exports.
root=fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root,'common')),fullfile(root,'stage1_point_mass'));
[cfg,p,tab]=impulse_grf_validation_fixture();
cfg.startsPerK=1; cfg.starts=1; cfg.localStarts=1; cfg.maxIterations=2; cfg.maxEvaluations=100;
cfg.useParallel=false; cfg.nodes=31; cfg.finalNodes=61; cfg.video=false;
cfg.model='stage1_point_mass'; cfg.method='scaled_mean_velocity_absolute_impulse_hermite';
cfg.simulationFunction='sim_cycle_stage1_point_mass';
folder=fullfile(root,'_logs','test',['hermite_smoke_' char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'))]);
mkdir(folder);
result=optimize_impulse_two_stage(p,tab,cfg);
assert(all(ismember({'bestVariables','final','stageStats'},fieldnames(result))));
assert(~isfield(result,'history') && ~isfield(result,'discreteGroups'));
save_run_outputs(result,folder);
execution=struct('status','smoke_test','startedAt','','finishedAt','',...
 'totalElapsed',result.elapsed,'matlabVersion',version,'invocationDirectory',pwd,...
 'projectRoot',root,'entryPoint',mfilename,'callChain',{{mfilename}},'gitCommit','');
write_optimization_report(result,folder,execution,cfg);
fprintf('Hermite optimization/output smoke test saved: %s\n',folder);
end
