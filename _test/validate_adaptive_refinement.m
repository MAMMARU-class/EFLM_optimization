function fixture=validate_adaptive_refinement(source)
% Reuse a real completed small phase2, then test an eight-corner subdivision.
root=fileparts(fileparts(mfilename('fullpath'))); addpath(root);
fixture=fullfile(root,'_logs','test',['adaptive_refinement_' char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'))]);
mkdir(fixture); copyfile(fullfile(source,'phase1'),fullfile(fixture,'phase1'));
copyfile(fullfile(source,'phase2'),fullfile(fixture,'phase2'));
s=load(fullfile(source,'adaptive_settings.mat'));
s.base.vx0=[2.2 .02 3]; s.base.thetaL=[deg2rad(47.5) deg2rad(.5) 3]; s.base.r0=[.3 .002 3];
s.spacing=[.02 deg2rad(.5) .002];
cases=dir(fullfile(fixture,'phase2','cases','*.mat'));
for i=1:numel(cases)
 path=fullfile(cases(i).folder,cases(i).name); item=load(path);
 record=item.record; record.cellSpacing=s.spacing; save(path,'record');
end
s.base.maxIterations=3; s.base.maxEvaluations=150;
s.plan.starts_phase3=1; s.plan.pass_phase3=100; s.plan.n_phase3=11; s.plan.m_phase3=7;
save(fullfile(fixture,'adaptive_settings.mat'),'-struct','s');
[rows,~]=run_stage1_basic(struct('resumeFolder',fixture,'generatePhaseOutputs',false));
p=load(fullfile(fixture,'phase3.1','settings.mat'));
new=p.tasks(cellfun(@(t)isempty(t.retained),p.tasks));
basics=cell2mat(cellfun(@(t)t.basic,new','UniformOutput',false));
assert(size(unique(basics,'rows'),1)==9); % Eight corners plus center after n change.
old=readtable(fullfile(source,'phase2','summary.csv'));
assert(min(rows.cost(rows.feasible==1))<=min(old.cost(old.feasible==1))+1e-12);
for i=1:numel(new)
 assert(all(abs(new{i}.spacing-s.spacing/3)<1e-12));
 assert(all(new{i}.cfg.initial>=new{i}.cfg.lower-1e-12 & new{i}.cfg.initial<=new{i}.cfg.upper+1e-12));
end
% Real saved parents with different n must reconstruct independently.
phase=fullfile(source,'phase3.2'); data=load(fullfile(phase,'settings.mat'));
rows=readtable(fullfile(phase,'summary.csv')); cfg=data.cfg; cfg.topResults=1; cfg.video=false;
export_basic_ranked(phase,rows,cfg,data.p,basic_geometry_cache(data.p));
plot_basic_grid_search_results(phase,fullfile(phase,'grid_search_result'));
analyze_basic_grf_pca(phase);
ranked=sortrows(rows(rows.feasible==1,:),{'cost','caseId'});
result=reconstruct_basic_case(phase,ranked.caseId(1));
result.settings.fps=2; result.settings.minimumVideoSeconds=.5;
result.settings.cycles=1; result.settings.playbackSpeed=1;
render_eflm_video(result,phase,'validation.mp4');
assert(isfile(fullfile(phase,'animation','validation.mp4')));
assert(numel(dir(fullfile(phase,'grid_search_result','*.png')))==29);
fprintf('Refinement and output validation passed: %s\n',fixture);
end
