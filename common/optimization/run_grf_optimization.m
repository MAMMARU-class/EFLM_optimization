function [result,folder]=run_grf_optimization(kind,overrides,root)
% Shared execution, atomic checkpoints and outputs for both curve families.
validateattributes(overrides,{'struct'},{'scalar'});
resumeFolder=''; finishBroadEarly=false;
if isfield(overrides,'resumeFolder')
 resumeFolder=char(overrides.resumeFolder); overrides=rmfield(overrides,'resumeFolder');
end
if isfield(overrides,'finishBroadEarly')
 finishBroadEarly=overrides.finishBroadEarly; overrides=rmfield(overrides,'finishBroadEarly');
 validateattributes(finishBroadEarly,{'logical'},{'scalar'});
end
if isempty(resumeFolder)
 if finishBroadEarly, error('EFLM:ResumeSettings','finishBroadEarly requires resumeFolder.'); end
 [cfg,p]=grf_optimization_settings(kind,overrides);
 folder=fullfile(root,'_logs',cfg.model,char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')));
 mkdir(folder); save(fullfile(folder,'settings.mat'),'cfg','p');
 copyfile(fullfile(root,'common'),fullfile(folder,'source','common'));
 copyfile(fullfile(root,'stage1_point_mass'),fullfile(folder,'source','stage1_point_mass'));
 for entry={'run_stage1_bspline.m','run_stage1_optimization.m'}
  copyfile(fullfile(root,entry{1}),fullfile(folder,'source',entry{1}));
 end
else
 if ~isempty(fieldnames(overrides))
  error('EFLM:ResumeSettings','Pass only resumeFolder and finishBroadEarly; saved settings are authoritative.');
 end
 saved=load(fullfile(resumeFolder,'settings.mat'),'cfg','p'); cfg=saved.cfg; p=saved.p;
 if cfg.layoutVersion==2 && strcmp(kind,'hermite') && ~finishBroadEarly
  [result,folder]=main_stage1(struct('resumeFolder',resumeFolder)); return;
 end
 legacyBspline=cfg.layoutVersion==3 && strcmp(kind,'bspline');
 if ~legacyBspline && (cfg.layoutVersion~=4 || ~strcmp(cfg.curveType,kind))
  error('EFLM:ResumeSettings',...
   'This checkpoint uses another formulation. Resume old runs using the source snapshot in that log folder.');
 end
 folder=resumeFolder;
end
fprintf('Run folder: %s\n',folder);
if cfg.layoutVersion==4
 fprintf('%s | independent stance time | %d controls | mu=%.3g\n',kind,cfg.controlCount,p.frictionCoefficient);
else
 fprintf('Resuming legacy %s with saved settings.\n',kind);
end
tab=build_exact_geometry_cache(p); prepare_optimization_pool(cfg);
warningGuard=onCleanup(@()bspline_warning_mode('restore',cfg)); %#ok<NASGU>
try
 bspline_warning_mode('disable',cfg);
 result=optimize_bspline_two_stage(p,tab,cfg,folder,finishBroadEarly);
catch ME
 fid=fopen(fullfile(folder,'report.txt'),'w');
 fprintf(fid,'Optimization stopped\n%s\n',getReport(ME,'extended')); fclose(fid);
 fprintf('Optimization stopped. Details: %s\n',fullfile(folder,'report.txt'));
 entry='run_stage1_optimization'; if strcmp(kind,'bspline'), entry='run_stage1_bspline'; end
 fprintf('Resume: %s(struct(''resumeFolder'',''%s''))\n',entry,strrep(folder,'''',''''''));
 result=[]; return;
end
if cfg.layoutVersion==3
 old=result.final.body.bspline;
 result.final.body.bspline.absoluteImpulse=result.final.body.Ts*bspline_absolute_area(old.cx,old.knotsX);
end
save(fullfile(folder,'result.mat'),'result','-v7');
export=result; export.final.body=rmfield(export.final.body,{'horizontalGRF','verticalGRF'});
save_run_outputs(export,folder);
if cfg.layoutVersion==4, g=result.final.body.grf; else, g=result.final.body.bspline; end
for axis={'x','z'}
 label=axis{1}; c=g.(['c' label]); amplitude=g.(['A' label]);
 writetable(table((0:numel(c)-1)',c,amplitude*c,...
  'VariableNames',{'controlIndex','coefficient','forceCoefficientN'}),...
  fullfile(folder,[kind '_' label '_controls.csv']));
end
fid=fopen(fullfile(folder,'report.txt'),'w');
fprintf(fid,'%s, layout %d\n',kind,cfg.layoutVersion);
fprintf(fid,'Variables: %d; equalities: 2; effective dimension: %d\n',numel(cfg.initial),numel(cfg.initial)-2);
fprintf(fid,'Broad: %d/%d starts at %d nodes; narrow: %d at %d nodes; checkpoint: %d starts\n',...
 result.stageStats.global.starts,cfg.broadStarts,cfg.nodes,result.stageStats.local.starts,cfg.narrowNodes,cfg.save_interval);
fprintf(fid,'Status: %s\nCost: %.12g\nSpeed: %.12g\nCoT: %.12g\nMax violation: %.12g\n',...
 result.status,result.final.objective,result.final.body.speed,result.final.cot,result.final.maxViolation);
fprintf(fid,'Broad seconds: %.9g\nNarrow seconds: %.9g\n',result.stageStats.global.elapsed,result.stageStats.local.elapsed);
fclose(fid);
if cfg.video
 render_eflm_video(result,folder);
 render_cycle_dashboard_video(result,folder);
end
fprintf('Saved: %s\n',folder);
end
