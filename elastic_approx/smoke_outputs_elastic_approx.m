function folder = smoke_outputs_elastic_approx()
% Verify serialization, plotting and video while optimizer toolboxes are absent.
% This is explicitly a diagnostic trajectory, NOT an optimization result.
root=fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root,'common')),genpath(fullfile(root,'elastic_approx')));
p=robot_parameters(); cfg=optimization_settings(); tab=build_geometry_cache(p);
cfg.cycles=1;
result=struct('parameters',p,'settings',cfg,'bestVariables',cfg.initial,...
 'status','diagnostic_only_NOT_optimized','valid',false,...
 'flightVisualization','Ballistic body only; leg hidden in flight');
result.final=sim_cycle_elastic_approx(cfg.initial,p,tab,cfg,cfg.finalNodes);
folder=fullfile(root,'_logs','elastic_approx',char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')));
mkdir(folder); save(fullfile(folder,'diagnostic_result.mat'),'result','-v7.3');
save_run_outputs(result,folder); render_eflm_video(result,folder);
fprintf('DIAGNOSTIC ONLY (not optimized): %s\n',folder);
fprintf('max angle lower/upper violations: %.6g %.6g rad\n',...
 max(p.jointLower-result.final.angles(:,1:2),[],'all'),...
 max(result.final.angles(:,1:2)-p.jointUpper,[],'all'));
end
