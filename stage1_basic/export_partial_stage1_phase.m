function summary=export_partial_stage1_phase(folder)
% Export the completed cases of an interrupted adaptive phase without resuming optimization.
root=fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root,'common')),fullfile(root,'stage1_basic'));
assert(strcmp(version('-release'),'2026b'),'Use MATLAB R2026b');
settingsFile=fullfile(folder,'settings.mat');
if ~isfile(settingsFile), error('EFLM:MissingSettings','Missing %s',settingsFile); end
saved=load(settingsFile,'cfg','p','meta');
meta=saved.meta;
checkpoint=fullfile(folder,'checkpoint.mat');
if isfile(checkpoint)
 state=load(checkpoint,'meta'); meta=state.meta;
end
files=dir(fullfile(folder,'cases','*.mat'));
if isempty(files), error('EFLM:NoCases','No saved cases in %s',folder); end
rows=cell(numel(files),1);
for i=1:numel(files)
 item=load(fullfile(files(i).folder,files(i).name),'record');
 rows{i}=basic_case_summary(item.record);
end
summary=vertcat(rows{:});
summary=sortrows(summary,'caseId');
writetable(summary,fullfile(folder,'summary.csv'));
meta.status='partial_results_exported';
if isfield(meta,'error'), meta=rmfield(meta,'error'); end
write_basic_report(folder,summary,saved.cfg,meta);
fprintf('Saved cases: %d/%d; feasible: %d. Exporting partial phase results.\n',...
 height(summary),meta.totalCases,nnz(summary.feasible));
tab=basic_geometry_cache(saved.p);
rankedCount=min(saved.cfg.topResults,nnz(summary.feasible));
rankedReady=true;
for rank=1:rankedCount
 rankedReady=rankedReady && isfile(fullfile(folder,sprintf('report_no%d.txt',rank))) && ...
  isfile(fullfile(folder,'plots',sprintf('no%d_parameters.png',rank)));
 if saved.cfg.video
  rankedReady=rankedReady && isfile(fullfile(folder,'animation',sprintf('no%d_parameters.mp4',rank)));
 end
end
if ~rankedReady, export_basic_ranked(folder,summary,saved.cfg,saved.p,tab); end
plot_basic_grid_search_results(folder,fullfile(folder,'grid_search_result'));
analyze_basic_grf_pca(folder);
fprintf('Partial phase outputs saved to %s\n',folder);
end
