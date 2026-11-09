function validate_stage1_basic()
% Small integration check: fixed variables, all grid/k outcomes, reconstruction.
root=fileparts(fileparts(mfilename('fullpath')));
addpath(root,genpath(fullfile(root,'common')),fullfile(root,'stage1_basic'));
c=configure_stage1_basic(struct);
assert(prod(cellfun(@numel,c.gridAxes))==1000);
assert(isempty(intersect(c.freeIndices,c.parameterGroups.basic)));
o=struct('vx0',[2.1 2.2 2],'thetaL',[.87 .87 1],'r0',[.3 .3 1],...
 'restrictGrfK',true,'grfKSelection',[8 9],'startsPerK',1,...
 'maxIterations',1,'maxEvaluations',65,'useParallel',false,'topResults',0);
[summary,folder]=run_stage1_basic(o);
assert(height(summary)==4 && all(summary.starts==1));
assert(isequal(summary.k',[8 9 8 9]));
assert(max(abs(summary.vx0'-[2.1 2.1 2.2 2.2]))<1e-14);
for id=1:4
 item=load(fullfile(folder,'cases',sprintf('%06d.mat',id)));
 assert(isfield(item,'record') && ~isfield(item.record,'history'));
 if ~isempty(item.record.variables)
  assert(isequal(item.record.variables(1:3),item.record.basic));
 end
end
assert(~isfolder(fullfile(folder,'animation')) && ~isfolder(fullfile(folder,'plots')));
% A saved old candidate supplies a nontrivial geometry/GRF integration fixture.
addpath(fullfile(root,'stage1_point_mass'));
[previous,p,~]=impulse_grf_validation_fixture();
a=previous.initial;
o.vx0=[a(1) a(1) 1]; o.thetaL=[a(2) a(2) 1]; o.r0=[a(3) a(3) 1];
o.grfKSelection=previous.grfK; o.startsPerK=2; o.maxIterations=20; o.maxEvaluations=1500;
o.useParallel=true; o.parallelWorkers=2;
cfg=configure_stage1_basic(o); cfg.initial=a; cfg.topResults=1;
cfg.fps=2; cfg.cycles=1; cfg.playbackSpeed=1; cfg.minimumVideoSeconds=.2;
tab=basic_geometry_cache(p); prepare_optimization_pool(cfg);
record=optimize_basic_case(a(1:3),cfg.grfK,1,p,tab,cfg); record.gridId=1;
assert(record.starts==2 && ~isempty(record.variables));
assert(isequal(record.variables(1:3),a(1:3)));
dest=fullfile(root,'_logs','test',['stage1_basic_' char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'))]);
mkdir(dest); mkdir(fullfile(dest,'cases')); save(fullfile(dest,'settings.mat'),'cfg','p');
save(fullfile(dest,'cases','000001.mat'),'record');
result=reconstruct_basic_case(dest,1);
assert(abs(result.final.objective-record.cost)<1e-10);
if record.feasible
 export_basic_ranked(dest,basic_case_summary(record),cfg,p,tab);
else
 % Exercise media output for the honest infeasible candidate, without ranking it.
 mkdir(fullfile(dest,'plots'));
 plot_basic_result(result,fullfile(dest,'plots','no1_parameters.png'));
 render_eflm_video(result,dest,'no1_parameters.mp4');
end
files=dir(fullfile(dest,'plots')); assert(nnz(~[files.isdir])==1);
files=dir(fullfile(dest,'animation')); assert(nnz(~[files.isdir])==1);
assert(isfile(fullfile(dest,'plots','no1_parameters.png')));
assert(isfile(fullfile(dest,'animation','no1_parameters.mp4')));
fprintf('stage1_basic checks passed. Grid: %s\nMedia: %s\n',folder,dest);
end
