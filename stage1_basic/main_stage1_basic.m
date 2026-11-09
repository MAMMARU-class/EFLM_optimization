function [summary,folder] = main_stage1_basic(options)
if nargin<1, options=struct; end
root=fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root,'common')),fullfile(root,'stage1_basic'));
assert(strcmp(version('-release'),'2026b'),'Use MATLAB R2026b');
assert(exist('fmincon','file')==2 && exist('MultiStart','class')==8,...
 'Optimization Toolbox and Global Optimization Toolbox are required.');
cfg=configure_stage1_basic(options); p=robot_parameters(); tab=basic_geometry_cache(p);
prepare_optimization_pool(cfg);
warningCleanup=suppress_basic_rank_warning(cfg); %#ok<NASGU>
[v,t,r]=ndgrid(cfg.gridAxes{:}); grid=[v(:),t(:),r(:)];
total=size(grid,1)*numel(cfg.grfKValues);
folder=fullfile(root,'_logs','stage1_basic',char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')));
mkdir(folder); mkdir(fullfile(folder,'cases'));
meta=struct('startedAt',char(datetime('now','TimeZone','Asia/Tokyo')), ...
 'matlabVersion',version,'status','running','totalCases',total,'elapsed',0);
[~,commit]=system('git rev-parse HEAD'); meta.gitCommit=strtrim(commit);
save(fullfile(folder,'settings.mat'),'cfg','p','meta','grid');
% A single source copy makes reconstruction independent of later code edits.
copyfile(fullfile(root,'common'),fullfile(folder,'source','common'));
copyfile(fullfile(root,'stage1_basic'),fullfile(folder,'source','stage1_basic'));
copyfile(fullfile(root,'run_stage1_basic.m'),fullfile(folder,'source','run_stage1_basic.m'));
summary=table; caseId=0; watch=tic;
fprintf('stage1_basic: %d basic grid points x %d k values x %d starts = %.0f local runs\n',...
 size(grid,1),numel(cfg.grfKValues),cfg.startsPerK,total*cfg.startsPerK);
write_basic_report(folder,summary,cfg,meta);
try
 gridCount=size(grid,1);
 for gridId=1:gridCount
  fprintf('\n[%d/%d] vx0=%.5g, thetaL=%.5g rad, r0=%.5g\n',...
   gridId,gridCount,grid(gridId,:));
  for k=cfg.grfKValues
   caseId=caseId+1;
   record=optimize_basic_case(grid(gridId,:),k,caseId,p,tab,cfg);
   record.gridId=gridId;
   % Save only each completed combination's final candidate and outcome.
   save(fullfile(folder,'cases',sprintf('%06d.mat',caseId)),'record','-v7');
   row=basic_case_summary(record); summary=[summary;row]; %#ok<AGROW>
   if caseId==1
    writetable(row,fullfile(folder,'summary.csv'));
   else
    writetable(row,fullfile(folder,'summary.csv'),'WriteMode','append','WriteVariableNames',false);
   end
   meta.elapsed=toc(watch); write_basic_report(folder,summary,cfg,meta);
   fprintf(['k = %d: %s | v_ave = %.5g | CoT = %.5g | cost=%.5g | ' ...
    'violation=%.3g | %.1f s\n'],...
    k,record.status,record.speed,record.cot,record.cost,...
    record.maxViolation,record.elapsed);
  end
 end
 meta.status='optimization_completed'; meta.elapsed=toc(watch);
 write_basic_report(folder,summary,cfg,meta);
 export_basic_ranked(folder,summary,cfg,p,tab);
 meta.status='completed'; meta.elapsed=toc(watch);
 save(fullfile(folder,'settings.mat'),'meta','-append');
 write_basic_report(folder,summary,cfg,meta);
catch ME
 meta.status='interrupted_or_failed'; meta.elapsed=toc(watch); meta.error=ME.message;
 save(fullfile(folder,'settings.mat'),'meta','-append');
 write_basic_report(folder,summary,cfg,meta); rethrow(ME);
end
fprintf('stage1_basic results: %s\n',folder);
end
