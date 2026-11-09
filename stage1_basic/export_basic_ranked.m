function export_basic_ranked(folder,summary,cfg,p,tab)
% Global top N across all (basic,k) cases. No promotion or pruning.
if isempty(summary) || cfg.topResults==0, return; end
ranked=sortrows(summary(summary.feasible==1,:),{'cost','caseId'});
count=min(cfg.topResults,height(ranked));
if count==0, fprintf('No feasible combinations: no ranked plots/videos.\n'); return; end
mkdir(fullfile(folder,'plots'));
if cfg.video, mkdir(fullfile(folder,'animation')); end
for rank=1:count
 id=ranked.caseId(rank);
 item=load(fullfile(folder,'cases',sprintf('%06d.mat',id)),'record'); record=item.record;
 caseCfg=basic_record_config(record,cfg);
 r=sim_cycle_stage1_basic(record.variables,p,tab,caseCfg,caseCfg.nodes);
 result=struct('final',r,'parameters',p,'settings',caseCfg,'bestVariables',record.variables,...
  'valid',record.feasible,'status',record.status);
 fprintf('Exporting rank %d/%d (case %d): PNG, video=%d\n',rank,count,id,cfg.video);
 write_basic_rank_report(result,folder,rank,id);
 plot_basic_result(result,fullfile(folder,'plots',sprintf('no%d_parameters.png',rank)));
 if cfg.video, render_eflm_video(result,folder,sprintf('no%d_parameters.mp4',rank)); end
end
end
