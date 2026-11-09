function [options,plan,control]=adaptive_phase_settings(options)
% Keep phase settings separate from the existing physical-model options.
defaults=struct('starts_phase1',50,'n_phase1',7,'k_phase1',4,'m_phase1',5,...
 'starts_phase2',100,'pass_phase2',50,'n_phase2',9,'m_phase2',6,...
 'starts_phase3',[200 200],'pass_phase3',[25 10],'n_phase3',[11 13],...
 'm_phase3',[7 9],'kRadiusFraction',.10,'refinementRadius',.20,...
 'resumeFolder','','stopAfterPhase',Inf,'generatePhaseOutputs',true,'appendPhase3',[]);
for name=fieldnames(defaults)'
 key=name{1}; if isfield(options,key), defaults.(key)=options.(key); options=rmfield(options,key); end
end
plan=defaults;
control=struct('resumeFolder',plan.resumeFolder,'stopAfterPhase',plan.stopAfterPhase,...
 'generatePhaseOutputs',plan.generatePhaseOutputs,'appendPhase3',plan.appendPhase3);
plan=rmfield(plan,fieldnames(control));
for name={'starts_phase1','starts_phase2','starts_phase3'}
 validateattributes(plan.(name{1}),{'numeric'},{'vector','integer','positive','finite'});
end
for name={'n_phase1','n_phase2','n_phase3'}
 validateattributes(plan.(name{1}),{'numeric'},{'vector','integer','>=',5,'finite'});
end
for name={'m_phase1','m_phase2','m_phase3'}
 validateattributes(plan.(name{1}),{'numeric'},{'vector','integer','>=',3,'finite'});
end
for name={'pass_phase2','pass_phase3'}
 validateattributes(plan.(name{1}),{'numeric'},{'vector','>',0,'<=',100,'finite'});
end
for name={'starts_phase1','n_phase1','k_phase1','m_phase1','starts_phase2','pass_phase2','n_phase2','m_phase2'}
 validateattributes(plan.(name{1}),{'numeric'},{'scalar'});
end
validateattributes(plan.k_phase1,{'numeric'},{'integer','>=',3,'<=',plan.n_phase1-2});
count=numel(plan.pass_phase3);
for name={'starts_phase3','n_phase3','m_phase3'}
 value=plan.(name{1});
 if isscalar(value), value=repmat(value,1,count); end
 if numel(value)~=count, error('EFLM:PhaseSettings','Phase3 arrays must have equal lengths (or be scalar).'); end
 plan.(name{1})=value;
end
validateattributes(plan.kRadiusFraction,{'numeric'},{'scalar','positive','<=',1,'finite'});
validateattributes(plan.refinementRadius,{'numeric'},{'scalar','positive','<=',1,'finite'});
validateattributes(control.stopAfterPhase,{'numeric'},{'scalar','positive'});
if isfinite(control.stopAfterPhase) && fix(control.stopAfterPhase)~=control.stopAfterPhase
 error('EFLM:PhaseSettings','stopAfterPhase must be an integer or Inf.');
end
validateattributes(control.generatePhaseOutputs,{'logical'},{'scalar'});
end
