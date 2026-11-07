function [result,folder] = main_stage1(overrides)
% Stage 1 point-mass optimization. Requires MATLAB R2026b.
if nargin<1, overrides=struct; end
validateattributes(overrides,{'struct'},{'scalar'},mfilename,'overrides');
assert(strcmp(version('-release'),'2026b'),'Use MATLAB R2026b');
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
if exist('fmincon','file')~=2 || exist('MultiStart','class')~=8
 % A running MATLAB session does not automatically acquire paths for
 % products installed after that session was started. Rebuild the default
 % R2026b path once, then restore this project's model paths.
 restoredefaultpath;
 rehash toolboxcache;
 addpath(genpath(fullfile(root,'common')));
end
if exist('fmincon','file')~=2 || exist('MultiStart','class')~=8
 error('EFLM:MissingToolbox',...
  ['Optimization Toolbox or Global Optimization Toolbox is unavailable in R2026b. ' ...
   'Restart MATLAB after installing the products.']);
end
addpath(genpath(fullfile(root,'common')));
p=robot_parameters(); cfg=optimization_settings();
cfg=apply_stage1_overrides(cfg,overrides);
model='stage1_point_mass';
modelFolder=fullfile(root,model);
addpath(genpath(modelFolder));
cfg.method='scaled_initial_velocity';
cfg.simulationFunction='sim_cycle_stage1_point_mass';
cfg.cacheFunction='build_exact_geometry_cache';
validateattributes(cfg.lower,{'numeric'},{'row','numel',6,'real','finite'});
validateattributes(cfg.upper,{'numeric'},{'row','numel',6,'real','finite'});
validateattributes(cfg.initial,{'numeric'},{'row','numel',6,'real','finite'});
validateattributes(cfg.lower(5:6),{'numeric'},{'positive'});
validateattributes(cfg.w1,{'numeric'},{'scalar','real','finite','nonnegative'});
validateattributes(cfg.w2,{'numeric'},{'scalar','real','finite','nonnegative'});
validateattributes(cfg.targetSpeed,{'numeric'},{'scalar','real','finite','positive'});
validateattributes(cfg.targetCoT,{'numeric'},{'scalar','real','finite','positive'});
if cfg.w1==0 && cfg.w2==0
 error('EFLM:ObjectiveWeights','At least one objective weight must be positive.');
end
if any(cfg.lower>=cfg.upper), error('EFLM:Bounds','Every lower bound must be below its upper bound.'); end
if any(cfg.initial<cfg.lower|cfg.initial>cfg.upper)
 error('EFLM:InitialPoint','Initial point must lie inside the broad search bounds.');
end
cfg.model=model;
folder=fullfile(root,'_logs',model,char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')));
mkdir(folder); diary(fullfile(folder,'run.log')); dc=onCleanup(@()diary('off'));
prepare_optimization_pool(cfg);
runTimer=tic;
stack=dbstack;
execution=struct;
execution.startedAt=char(datetime('now','TimeZone','Asia/Tokyo','Format','yyyy-MM-dd HH:mm:ss.SSS Z'));
execution.finishedAt=''; execution.totalElapsed=NaN; execution.status='running';
execution.matlabVersion=version; execution.invocationDirectory=pwd;
execution.projectRoot=root; execution.callChain=fliplr({stack.name});
if numel(stack)>1, execution.entryPoint=stack(2).name; else, execution.entryPoint=stack(1).name; end
% Snapshot code as well as settings: dirty changes must be reproducible too.
copyfile(fullfile(root,'common'),fullfile(folder,'source','common'));
copyfile(modelFolder,fullfile(folder,'source',model));
copyfile(fullfile(root,'run_stage1_optimization.m'),fullfile(folder,'source','run_stage1_optimization.m'));
[~,gitInfo]=system('git rev-parse HEAD');
execution.gitCommit=strtrim(gitInfo);
save(fullfile(folder,'settings.mat'),'p','cfg','gitInfo');
try
 tab=feval(cfg.cacheFunction,p);
 fprintf('R2026b | %s | mass %.9f kg | %d + %d MultiStart runs\n',model,p.m,cfg.starts,cfg.localStarts);
 result=optimize_two_stage(p,tab,cfg,folder);
 execution.status='completed'; execution.finishedAt=char(datetime('now','TimeZone','Asia/Tokyo','Format','yyyy-MM-dd HH:mm:ss.SSS Z'));
 execution.totalElapsed=toc(runTimer); result.execution=execution; result.gitCommit=execution.gitCommit;
 save(fullfile(folder,'result.mat'),'result','-v7.3');
 save_run_outputs(result,folder);
 if cfg.video, render_eflm_video(result,folder); end
 execution.totalElapsed=toc(runTimer); execution.finishedAt=char(datetime('now','TimeZone','Asia/Tokyo','Format','yyyy-MM-dd HH:mm:ss.SSS Z'));
 result.execution=execution; save(fullfile(folder,'result.mat'),'result','-v7.3');
 write_optimization_report(result,folder,execution,cfg);
 disp(readtable(fullfile(folder,'summary.csv')));
 fprintf('Saved: %s\n',folder);
catch ME
 execution.status='failed'; execution.finishedAt=char(datetime('now','TimeZone','Asia/Tokyo','Format','yyyy-MM-dd HH:mm:ss.SSS Z'));
 execution.totalElapsed=toc(runTimer);
 write_optimization_report([],folder,execution,cfg,ME);
 save(fullfile(folder,'failure.mat'),'ME','p','cfg');
 rethrow(ME);
end
end
