function [summary,folder]=main_stage1_adaptive(options)
% Resumable feasibility screening, k screening, and local grid refinement.
root=fileparts(fileparts(mfilename('fullpath')));
[base,plan,control]=adaptive_phase_settings(options);
assert(strcmp(version('-release'),'2026b'),'Use MATLAB R2026b');
if isempty(control.resumeFolder)
 if ~isempty(control.appendPhase3), error('EFLM:PhaseSettings','appendPhase3 requires resumeFolder.'); end
 folder=fullfile(root,'_logs','stage1_basic',char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')));
 mkdir(folder);
 % Validate physical settings before any expensive solver work.
base=phaseOptions(base,plan,1); cfg=configure_stage1_basic(base);
 p=robot_parameters();
 [v,t,r]=ndgrid(cfg.gridAxes{:}); grid=[v(:),t(:),r(:)];
 spacing=2*cfg.gridSpecs(:,2)'./max(1,cfg.gridSpecs(:,3)'-1);
 spacing(cfg.gridSpecs(:,3)'==1)=0;
 save(fullfile(folder,'adaptive_settings.mat'),'base','plan','p','grid','spacing');
 copyfile(fullfile(root,'common'),fullfile(folder,'source','common'));
 copyfile(fullfile(root,'stage1_basic'),fullfile(folder,'source','stage1_basic'));
 copyfile(fullfile(root,'run_stage1_basic.m'),fullfile(folder,'source','run_stage1_basic.m'));
else
 folder=char(control.resumeFolder);
 if ~isfile(fullfile(folder,'adaptive_settings.mat')) && isfile(fullfile(fileparts(folder),'adaptive_settings.mat'))
  folder=fileparts(folder);
 end
 s=load(fullfile(folder,'adaptive_settings.mat')); base=s.base; plan=s.plan;
 p=s.p; grid=s.grid; spacing=s.spacing;
 if ~isempty(control.appendPhase3)
  last=fullfile(folder,phaseLabel(2+numel(plan.pass_phase3)),'settings.mat');
  if ~isfile(last), error('EFLM:PhaseSettings','Complete the saved phases before appending.'); end
  state=load(last,'meta');
  checkpoint=fullfile(fileparts(last),'checkpoint.mat');
  if isfile(checkpoint), state=load(checkpoint,'meta'); end
  if ~strcmp(state.meta.status,'completed'), error('EFLM:PhaseSettings','Complete the last phase before appending.'); end
  extra=control.appendPhase3;
  if ~isstruct(extra) || ~isscalar(extra) || ~all(isfield(extra,{'starts','pass','n','m'}))
   error('EFLM:PhaseSettings','appendPhase3 requires starts, pass, n, m.');
  end
  names={'starts','pass','n','m'};
  for name=names
   key=[name{1} '_phase3']; plan.(key)=[plan.(key)(:) ; extra.(name{1})(:) ]';
  end
  % Validate the extended schedule using the same rules as a new run.
  [~,plan]=adaptive_phase_settings(plan);
  save(fullfile(folder,'adaptive_settings.mat'),'plan','-append');
 end
 fprintf('Resuming %s using its saved search settings.\n',folder);
end
tab=basic_geometry_cache(p); cfg=configure_stage1_basic(phaseOptions(base,plan,1));
prepare_optimization_pool(cfg); cleanup=suppress_basic_rank_warning(cfg); %#ok<NASGU>
records={}; summary=table;
for phase=1:min(2+numel(plan.pass_phase3),control.stopAfterPhase)
 label=phaseLabel(phase); destination=fullfile(folder,label);
 if isfile(fullfile(destination,'settings.mat'))
  s=load(fullfile(destination,'settings.mat')); cfg=s.cfg; tasks=s.tasks; meta=s.meta;
  if isfile(fullfile(destination,'checkpoint.mat'))
   state=load(fullfile(destination,'checkpoint.mat'),'meta'); meta=state.meta;
  end
 else
  cfg=configure_stage1_basic(phaseOptions(base,plan,phase));
  if phase==1
   cfg.initial(4)=1; cfg.lower(4)=1; cfg.upper(4)=1;
   cfg.freeIndices=setdiff(cfg.freeIndices,4);
  end
  tasks=makeTasks(phase,grid,spacing,records,cfg,plan);
  if ~isfolder(destination), mkdir(destination); end
  mkdir(fullfile(destination,'cases'));
  [~,commit]=system('git rev-parse HEAD');
  meta=struct('startedAt',char(datetime('now','TimeZone','Asia/Tokyo')), ...
   'matlabVersion',version,'status','running','totalCases',numel(tasks),...
   'elapsed',0,'gitCommit',strtrim(commit),'phase',label);
  save(fullfile(destination,'settings.mat'),'cfg','p','tasks','meta','plan','-v7');
 end
 fprintf('\n%s: %d combinations (including retained parents).\n',label,numel(tasks));
 if isfield(meta,'error'), meta=rmfield(meta,'error'); end
 records=cell(1,numel(tasks)); summary=table; previousElapsed=meta.elapsed; timer=tic;
 try
  for id=1:numel(tasks)
   path=fullfile(destination,'cases',sprintf('%06d.mat',id));
   if isfile(path)
    s=load(path,'record'); record=s.record;
   else
    task=tasks{id};
    if ~isempty(task.retained)
     record=task.retained;
     record.cellSpacing=task.spacing;
     record.starts=0; record.convergedStarts=0; record.iterations=0;
     record.functionEvaluations=0; record.elapsed=0;
    else
     fprintf('[%d/%d] vx0=%.5g thetaL=%.5g r0=%.5g k=%d\n',id,numel(tasks),task.basic,task.k);
     record=optimize_basic_case(task.basic,task.k,id,p,tab,task.cfg);
     record.configuration=task.cfg;
     record.originPhase=label; record.originCase=id;
     record.cellSpacing=task.spacing;
     record.parentOrigin=task.parentOrigin;
     fprintf('  %s | cost=%.6g | violation=%.3g | %.1f s\n',record.status,record.cost,record.maxViolation,record.elapsed);
    end
    record.caseId=id; record.gridId=task.gridId;
    record.kFraction=(record.k-1)/(record.configuration.grfXN-1);
    temporary=[path '.tmp']; save(temporary,'record','-v7'); movefile(temporary,path,'f');
   end
   records{id}=record; summary=[summary;basic_case_summary(record)]; %#ok<AGROW>
   meta.elapsed=previousElapsed+toc(timer);
   savePhaseMeta(destination,meta);
   % MAT case files are authoritative; a torn CSV can always be rebuilt.
  end
  if ~isempty(summary), writetable(summary,fullfile(destination,'summary.csv')); end
  meta.elapsed=previousElapsed+toc(timer); meta.status='optimization_completed';
  groups=rankGroups(records); bestTwo=cell(1,numel(groups));
  for g=1:numel(groups)
   ids=groups{g}; ids=ids(1:min(2,numel(ids)));
   bestTwo{g}=struct('basic',records{ids(1)}.basic,'caseIds',ids,...
    'kFractions',cellfun(@(r)r.kFraction,records(ids)),...
    'costs',cellfun(@(r)r.cost,records(ids)));
  end
  save(fullfile(destination,'selection.mat'),'bestTwo');
  savePhaseMeta(destination,meta);
  write_basic_report(destination,summary,cfg,meta);
  if control.generatePhaseOutputs && ~isempty(summary) && ~isfile(fullfile(destination,'outputs_complete.mat'))
   export_basic_ranked(destination,summary,cfg,p,tab);
   plot_basic_grid_search_results(destination,fullfile(destination,'grid_search_result'));
   analyze_basic_grf_pca(destination);
   outputsComplete=true; save(fullfile(destination,'outputs_complete.mat'),'outputsComplete');
  end
  meta.status='completed'; meta.elapsed=previousElapsed+toc(timer);
  savePhaseMeta(destination,meta);
  write_basic_report(destination,summary,cfg,meta);
 catch ME
  meta.elapsed=previousElapsed+toc(timer); meta.status='interrupted_or_failed'; meta.error=ME.message;
  savePhaseMeta(destination,meta);
  if ~isempty(summary), writetable(summary,fullfile(destination,'summary.csv')); end
  write_basic_report(destination,summary,cfg,meta); rethrow(ME);
 end
 writeOverview(folder,plan,phase,summary);
end
fprintf('Adaptive stage1 results: %s\n',folder);
end

function options=phaseOptions(base,plan,phase)
options=base; options.restrictGrfK=false;
if phase==1
 options.grfXN=plan.n_phase1; options.grfZN=plan.m_phase1;
 options.startsPerK=plan.starts_phase1; options.restrictGrfK=true; options.grfKSelection=plan.k_phase1;
elseif phase==2
 options.grfXN=plan.n_phase2; options.grfZN=plan.m_phase2; options.startsPerK=plan.starts_phase2;
else
 i=phase-2; options.grfXN=plan.n_phase3(i); options.grfZN=plan.m_phase3(i); options.startsPerK=plan.starts_phase3(i);
end
end

function tasks=makeTasks(phase,grid,spacing,previous,cfg,plan)
tasks={};
if phase==1
 for i=1:size(grid,1), tasks{end+1}=newTask(grid(i,:),cfg.grfKValues(1),cfg,spacing,i,[]); end %#ok<AGROW>
 return;
end
groups=rankGroups(previous);
if isempty(groups), return; end
percentage=plan.pass_phase2;
if phase>2, percentage=plan.pass_phase3(phase-2); end
selected=groups(1:ceil(numel(groups)*percentage/100));
if phase>2
 % Retain every feasible parent, with its original model and search bounds.
 [~,retentionOrder]=sort(cellfun(@(r)r.cost,previous));
 for i=retentionOrder
  r=previous{i}; if ~r.feasible, continue; end
  task=newTask(r.basic,r.k,r.configuration,r.cellSpacing/3,0,r);
  tasks{end+1}=task; %#ok<AGROW>
 end
end
for group=selected
 ids=group{1}; parent=previous{ids(1)};
 if phase==2
  children=parent.basic; childSpacing=spacing;
 else
  childSpacing=parent.cellSpacing/3;
  [x,y,z]=ndgrid([-1 1],[-1 1],[-1 1]);
  children=parent.basic+[x(:),y(:),z(:)].*childSpacing;
  previousN=plan.n_phase2; previousM=plan.m_phase2;
  if phase>3, previousN=plan.n_phase3(phase-3); previousM=plan.m_phase3(phase-3); end
  if cfg.grfXN~=previousN || cfg.grfZN~=previousM
   children=[parent.basic;children]; %#ok<AGROW>
  end
  children=unique(children,'rows','stable');
  limits=cfg.gridSpecs;
  children=children(all(children>=(limits(:,1)-limits(:,2))'-1e-12 & ...
   children<=(limits(:,1)+limits(:,2))'+1e-12,2),:);
 end
 if phase==2
  ks=cfg.grfKValues;
 else
  top=ids(1:min(2,numel(ids))); rho=cellfun(@(r)r.kFraction,previous(top));
  if phase==3
   valid=3:cfg.grfXN-2; fractions=(valid-1)/(cfg.grfXN-1);
   ks=valid(any(abs(fractions(:)-rho(:)')<=plan.kRadiusFraction+1e-12,2));
  else
   ks=[];
  end
  % Include each best location's nearest valid new knot, even for sparse grids.
  nearest=max(3,min(cfg.grfXN-2,round(1+rho*(cfg.grfXN-1))));
  ks=unique([ks(:);nearest(:)])';
 end
 for child=1:size(children,1)
  for k=ks
   caseCfg=cfg; caseCfg.grfK=k;
   origin=[];
   if phase>2
    [~,near]=min(abs(rho-(k-1)/(cfg.grfXN-1))); seed=previous{top(near)};
    caseCfg=basic_refinement_config(cfg,seed,k,children(child,:),plan.refinementRadius);
    origin=struct('phase',seed.originPhase,'caseId',seed.originCase);
   end
   task=newTask(children(child,:),k,caseCfg,childSpacing,0,[]); task.parentOrigin=origin;
   tasks{end+1}=task; %#ok<AGROW>
  end
 end
end
% Exact duplicate tasks from overlapping cells use the lowest-cost parent's seed.
keys=cellfun(@(t)sprintf('%.14g,%.14g,%.14g,%d,%d,%d',t.basic,t.k,t.cfg.grfXN,t.cfg.grfZN),tasks,'UniformOutput',false);
% Retained parents and new center searches must both survive.
for i=1:numel(tasks), keys{i}=[keys{i} sprintf(',retained=%d',~isempty(tasks{i}.retained))]; end
[~,keep]=unique(keys,'stable'); tasks=tasks(keep);
if isempty(tasks), return; end
basics=cell2mat(cellfun(@(t)t.basic,tasks','UniformOutput',false));
[~,~,ids]=unique(basics,'rows');
for i=1:numel(tasks), tasks{i}.gridId=ids(i); end
end

function groups=rankGroups(records)
groups={}; feasible=find(cellfun(@(r)r.feasible && isfinite(r.cost),records));
if isempty(feasible), return; end
basic=cell2mat(cellfun(@(r)r.basic,records(feasible)','UniformOutput',false));
[~,~,g]=unique(basic,'rows'); best=[];
for id=1:max(g)
 indices=feasible(g==id); costs=cellfun(@(r)r.cost,records(indices));
 [~,order]=sort(costs); indices=indices(order);
 % Distinct normalized k positions (parents may use different n).
 rho=cellfun(@(r)r.kFraction,records(indices)); [~,keep]=unique(round(rho,12),'stable');
 indices=indices(keep); groups{end+1}=indices; best(end+1)=records{indices(1)}.cost; %#ok<AGROW>
end
[~,order]=sort(best); groups=groups(order);
end

function t=newTask(basic,k,cfg,spacing,gridId,retained)
t=struct('basic',basic,'k',k,'cfg',cfg,'spacing',spacing,'gridId',gridId,...
 'retained',retained,'parentOrigin',[]);
end

function label=phaseLabel(index)
if index<3, label=sprintf('phase%d',index); else, label=sprintf('phase3.%d',index-2); end
end

function savePhaseMeta(folder,meta)
temporary=fullfile(folder,'checkpoint.tmp'); save(temporary,'meta','-v7');
movefile(temporary,fullfile(folder,'checkpoint.mat'),'f');
end

function writeOverview(folder,plan,phase,summary)
fid=fopen(fullfile(folder,'report.txt'),'w'); guard=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'Adaptive stage1 basic grid search\nLast completed phase: %s\n',phaseLabel(phase));
fprintf(fid,'Selection: ascending feasible cost, percentage of basic tuples, rounded up.\n');
fprintf(fid,'Phase2 pass: %.6g%%; Phase3 pass: %s%%\n',plan.pass_phase2,mat2str(plan.pass_phase3));
fprintf(fid,'k position: (k-1)/(n-1); phase3.1 radius: %.6g\n',plan.kRadiusFraction);
fprintf(fid,'Local half-width: %.6g x parent box width, clipped to physical bounds.\n',plan.refinementRadius);
feasible=0; if ~isempty(summary), feasible=sum(summary.feasible); end
fprintf(fid,'Final phase candidates: %d; feasible: %d\n',height(summary),feasible);
fprintf(fid,'Resume: run_stage1_basic(struct(''resumeFolder'',''%s''))\n',strrep(folder,'''',''''''));
end
