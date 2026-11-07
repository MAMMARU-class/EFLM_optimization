function folder = smoke_hermite_optimization()
% Small integration run: exercise categorical MultiStart, equality constraints,
% dense refinement, and final-result/CSV/report exports.
root=fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root,'common')),fullfile(root,'stage1_point_mass'));
p=robot_parameters(); cfg=optimization_settings();
cfg.starts=16; cfg.localStarts=2; cfg.maxIterations=3; cfg.maxEvaluations=100;
cfg.parallelWorkers=4; cfg.nodes=31; cfg.finalNodes=61; cfg.video=false;
cfg=configure_horizontal_grf(cfg);
cfg.model='stage1_point_mass'; cfg.method='scaled_initial_velocity';
cfg.simulationFunction='sim_cycle_stage1_point_mass';
folder=fullfile(root,'_logs','test',['hermite_smoke_' char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'))]);
mkdir(folder);
tab=build_exact_geometry_cache(p);
result=optimize_two_stage(p,tab,cfg);
assert(all(ismember({'bestVariables','final','stageStats'},fieldnames(result))));
assert(~isfield(result,'history') && ~isfield(result,'discreteGroups'));
save_run_outputs(result,folder);
execution=struct('status','smoke_test','startedAt','','finishedAt','',...
 'totalElapsed',result.elapsed,'matlabVersion',version,'invocationDirectory',pwd,...
 'projectRoot',root,'entryPoint',mfilename,'callChain',{{mfilename}},'gitCommit','');
write_optimization_report(result,folder,execution,cfg);
fprintf('Hermite optimization/output smoke test saved: %s\n',folder);
end
