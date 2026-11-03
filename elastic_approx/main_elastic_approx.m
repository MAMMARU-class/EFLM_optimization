function [result,folder] = main_elastic_approx(overrides)
% Entry point: result = main_elastic_approx(); Requires MATLAB R2026b.
if nargin<1, overrides=struct; end
assert(strcmp(version('-release'),'2026b'),'Use MATLAB R2026b');
root=fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root,'common')),genpath(fullfile(root,'elastic_approx')));
p=robot_parameters(); cfg=optimization_settings();
for key=fieldnames(overrides)', cfg.(key{1})=overrides.(key{1}); end
folder=fullfile(root,'_logs','elastic_approx',char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')));
mkdir(folder); diary(fullfile(folder,'run.log')); dc=onCleanup(@()diary('off'));
% Snapshot code as well as settings: dirty changes must be reproducible too.
copyfile(fullfile(root,'common'),fullfile(folder,'source','common'));
copyfile(fullfile(root,'elastic_approx'),fullfile(folder,'source','elastic_approx'));
[~,gitInfo]=system('git rev-parse HEAD');
save(fullfile(folder,'settings.mat'),'p','cfg','gitInfo');
tab=build_geometry_cache(p);
fprintf('R2026b | mass %.9f kg | %d + %d MultiStart runs\n',p.m,cfg.starts,cfg.localStarts);
try
 result=optimize_two_stage(p,tab,cfg,folder);
 save_run_outputs(result,folder);
 if cfg.video, render_eflm_video(result,folder); end
 disp(readtable(fullfile(folder,'summary.csv')));
 fprintf('Saved: %s\n',folder);
catch ME
 save(fullfile(folder,'failure.mat'),'ME','p','cfg');
 rethrow(ME);
end
end
