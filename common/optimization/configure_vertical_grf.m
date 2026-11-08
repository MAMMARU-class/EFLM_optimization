function cfg = configure_vertical_grf(cfg)
% Append normalized vertical knot values and independent endpoint slopes.
n=cfg.grfZN;
validateattributes(n,{'numeric'},{'scalar','integer','>=',3});
for name={'grfZBetaStart','grfZBetaEnd'}
 b=cfg.(name{1});
 validateattributes(b,{'numeric'},{'real','finite','size',[1 3]});
 if b(1)>=b(2) || b(3)<b(1) || b(3)>b(2)
  error('EFLM:VerticalBounds','Invalid vertical endpoint [lower, upper, initial].');
 end
end
bs=cfg.grfZBetaStart; be=cfg.grfZBetaEnd;
if bs(1)<0 || be(2)>0
 error('EFLM:VerticalBounds','Vertical start beta must be nonnegative and end beta nonpositive.');
end
cfg.grfZIndices=numel(cfg.initial)+(1:n);
f=sin(pi*(1:n-2)/(n-1)); f=f/sum(f);
cfg.lower=[cfg.lower,zeros(1,n-2),bs(1),be(1)];
cfg.upper=[cfg.upper,ones(1,n-2),bs(2),be(2)];
cfg.initial=[cfg.initial,f,bs(3),be(3)];
for i=2:n-1, cfg.variableNames{end+1}=sprintf('grfZValue%d',i); end
cfg.variableNames=[cfg.variableNames,{'grfZBetaStart','grfZBetaEnd'}];
cfg.shapeConstraintCount=cfg.shapeConstraintCount+n-1;
end
