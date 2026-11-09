function result = reconstruct_basic_case(folder,caseId)
% Recreate any saved combination for later plots or a new optimization seed.
root=fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root,'common')),fullfile(root,'stage1_basic'));
s=load(fullfile(folder,'settings.mat'),'cfg','p');
item=load(fullfile(folder,'cases',sprintf('%06d.mat',caseId)),'record');
record=item.record;
if isempty(record.variables), error('EFLM:NoTrajectory','This case has no evaluable final trajectory: %s',record.errorMessage); end
cfg=basic_record_config(record,s.cfg);
tab=basic_geometry_cache(s.p);
r=sim_cycle_stage1_basic(record.variables,s.p,tab,cfg,cfg.nodes);
result=struct('final',r,'parameters',s.p,'settings',cfg,'bestVariables',record.variables,...
 'valid',record.feasible,'status',record.status);
end
