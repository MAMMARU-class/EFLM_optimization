function cfg = configure_impulse_grf(cfg)
% Layout v2: basic state, timing, horizontal GRF, vertical GRF.
n=cfg.grfXN;
validateattributes(n,{'numeric'},{'scalar','integer','>=',5});
validateattributes(cfg.startsPerK,{'numeric'},{'scalar','finite','integer','positive'});
validateattributes(cfg.localStarts,{'numeric'},{'scalar','finite','integer','positive'});
validateattributes(cfg.restrictGrfK,{'logical'},{'scalar'});
if cfg.restrictGrfK
 validateattributes(cfg.grfKSelection,{'numeric'},{'vector','nonempty','finite','integer','>=',3,'<=',n-2});
 cfg.grfKValues=unique(cfg.grfKSelection(:)','stable');
else
 cfg.grfKValues=3:n-2;
end
for name={'grfXBetaStart','grfXBetaEnd'}
 b=cfg.(name{1});
 validateattributes(b,{'numeric'},{'real','finite','size',[1 3]});
 if b(1)>=b(2) || b(2)>0 || b(3)<b(1) || b(3)>b(2)
  error('EFLM:HorizontalBounds','%s must have lower < upper <= 0 and initial within bounds.',name{1});
 end
end
if cfg.lower(6)<0, error('EFLM:HorizontalImpulse','Ix must be nonnegative.'); end
if cfg.lower(1)<=0 || cfg.lower(3)<=0 || cfg.lower(4)<=0 || cfg.lower(5)<=0 || cfg.upper(5)>=1
 error('EFLM:Bounds','vx0, r0, stanceScale must be positive and 0 < duty < 1.');
end
cfg.index=struct('vx0',1,'thetaL',2,'r0',3,'stanceScale',4,'duty',5,'Ix',6,...
 'fx',7:n+3,'betaXStart',n+4,'betaXEnd',n+5);
cfg.parameterGroups=struct('basic',1:3,'timing',4:5,'horizontal',6:n+5);
cfg.variableNames={'vx0','thetaL','r0','stanceScale','duty','Ix'};
for i=1:n-3, cfg.variableNames{end+1}=sprintf('fxMagnitude%d',i); end
cfg.variableNames=[cfg.variableNames,{'grfXBetaStart','grfXBetaEnd'}];
bs=cfg.grfXBetaStart; be=cfg.grfXBetaEnd;
cfg.lower=[cfg.lower(1:6),repmat(1e-4,1,n-3),bs(1),be(1)];
cfg.upper=[cfg.upper(1:6),ones(1,n-3),bs(2),be(2)];
cfg.initial=[cfg.initial(1:6),ones(1,n-3)/(n-3),bs(3),be(3)];
cfg.grfK=cfg.grfKValues(1);
cfg.starts=cfg.startsPerK*numel(cfg.grfKValues);
cfg.shapeConstraintCount=2*(n-1);
cfg=configure_vertical_grf(cfg);
cfg.index.fz=cfg.grfZIndices(1:end-2);
cfg.index.betaZStart=cfg.grfZIndices(end-1); cfg.index.betaZEnd=cfg.grfZIndices(end);
cfg.parameterGroups.vertical=cfg.grfZIndices;
cfg.equalityConstraintCount=3;
cfg.initial=seed_horizontal_impulse(cfg.initial,cfg,false);
end
