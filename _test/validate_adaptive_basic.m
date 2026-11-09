function folder=validate_adaptive_basic()
% Small end-to-end test; never launches the production grid.
root=fileparts(fileparts(mfilename('fullpath'))); addpath(root);
o=struct('vx0',[2.2 0 1],'thetaL',[deg2rad(47.5) 0 1],'r0',[.30 0 1],...
 'starts_phase1',12,'starts_phase2',4,'starts_phase3',[2 2],...
 'pass_phase2',100,'pass_phase3',[100 100],'n_phase3',[9 11],'m_phase3',[6 7],...
 'useParallel',false,'generatePhaseOutputs',false,'video',false,...
 'maxIterations',50,'maxEvaluations',800,'stopAfterPhase',1);
[~,folder]=run_stage1_basic(o);
s=load(fullfile(folder,'phase1','settings.mat'));
assert(s.cfg.lower(4)==1 && s.cfg.upper(4)==1 && ~ismember(4,s.cfg.freeIndices));
file=fullfile(folder,'phase1','cases','000001.mat'); before=dir(file);
[~,again]=run_stage1_basic(struct('resumeFolder',folder,'generatePhaseOutputs',false));
assert(strcmp(folder,again)); after=dir(file); assert(before.datenum==after.datenum);
for label={'phase1','phase2','phase3.1','phase3.2'}
 assert(isfile(fullfile(folder,label{1},'settings.mat')));
 files=dir(fullfile(folder,label{1},'cases','*.mat'));
 for i=1:numel(files)
  item=load(fullfile(files(i).folder,files(i).name)); r=item.record;
  if isempty(r.variables), continue; end
  assert(all(r.variables>=r.configuration.lower-1e-8 & r.variables<=r.configuration.upper+1e-8));
 end
end
% Mapping and local-box normalization are tested even if the tiny search fails.
addpath(genpath(fullfile(root,'common')),fullfile(root,'stage1_basic'));
c=configure_stage1_basic(struct('grfXN',9,'grfZN',6,'startsPerK',3));
c.grfK=4; a=seed_horizontal_impulse(c.initial,c,false);
parent=struct('configuration',c,'variables',a,'k',4);
d=configure_stage1_basic(struct('grfXN',13,'grfZN',9,'startsPerK',7));
d=basic_refinement_config(d,parent,6,a(1:3),.2);
starts=basic_bounded_starts(d,a(1:3));
assert(all(starts>=d.lower-1e-12 & starts<=d.upper+1e-12,'all'));
for i=1:size(starts,1)
 h=horizontal_impulse_shape(starts(i,:),d);
 assert(abs(h.S0)<1e-10 && abs(h.normalizationResidual)<1e-10);
 assert(abs(sum(starts(i,d.index.fz))-1)<1e-10);
end
fprintf('Adaptive validation passed: %s\n',folder);
end
