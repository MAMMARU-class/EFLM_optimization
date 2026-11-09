function cfg = apply_stage1_overrides(cfg,overrides)
% Apply named [lower, upper, initial] decision-variable settings.
parameterNames={'vx0','thetaL','r0','stanceScale','duty','Ix'};

legacyNames={'lower','upper','initial','AmpH','v','Duty','grfN',...
 'grfBetaEndpoint','restrictGrfPairs','grfPairSelection','starts'};
for i=1:numel(legacyNames)
 if isfield(overrides,legacyNames{i})
  error('EFLM:LegacyParameterSetting',...
   ['Use vx0, thetaL, r0, stanceScale, duty, Ix and grfXN; ' ...
    'use startsPerK and restrictGrfK/grfKSelection. Obsolete setting: %s'],legacyNames{i});
 end
end

for i=1:numel(parameterNames)
 name=parameterNames{i};
 if ~isfield(overrides,name), continue; end
 values=overrides.(name);
 validateattributes(values,{'numeric'},...
  {'row','numel',3,'real','finite'},mfilename,['overrides.' name]);
 if values(1)>=values(2)
  error('EFLM:ParameterBounds',...
   'The lower bound must be below the upper bound for overrides.%s.',name);
 end
 if values(3)<values(1) || values(3)>values(2)
  error('EFLM:ParameterInitial',...
   'The initial value must lie within the bounds for overrides.%s.',name);
 end
 cfg.lower(i)=values(1);
 cfg.upper(i)=values(2);
 cfg.initial(i)=values(3);
 overrides=rmfield(overrides,name);
end

for key=fieldnames(overrides)'
 if ~isfield(cfg,key{1})
  error('EFLM:UnknownSetting','Unknown optimization setting: %s',key{1});
 end
 cfg.(key{1})=overrides.(key{1});
end
end
