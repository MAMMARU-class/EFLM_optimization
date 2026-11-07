function cfg = configure_horizontal_grf(cfg)
% n is fixed for one run; k and j are categorical choices in the broad stage.
n=cfg.grfN;
validateattributes(n,{'numeric'},{'scalar','integer','>=',4});
validateattributes(cfg.grfSlope,{'numeric'},{'real','finite','size',[1 3]});
if cfg.grfSlope(1)>=cfg.grfSlope(2) || cfg.grfSlope(3)<cfg.grfSlope(1) || cfg.grfSlope(3)>cfg.grfSlope(2)
 error('EFLM:GRFSlope','grfSlope must be [lower, upper, initial] in s^-1.');
end
if cfg.lower(1)<0, error('EFLM:GRFAmplitude','AmpH must be nonnegative.'); end
cfg.grfK=ceil(n/2); choices=setdiff(2:n-1,cfg.grfK); cfg.grfJ=choices(1);
cfg.variableNames={'AmpH','thetaL','v','Duty','r0','stanceScale'};
for i=1:n-4, cfg.variableNames{end+1}=sprintf('grfRatio%d',i); end
for i=1:n-1, cfg.variableNames{end+1}=sprintf('grfSlope%d',i); end
cfg.lower=[cfg.lower(1:6),repmat(1e-4,1,n-4),repmat(cfg.grfSlope(1),1,n-1)];
cfg.upper=[cfg.upper(1:6),repmat(1-1e-4,1,n-4),repmat(cfg.grfSlope(2),1,n-1)];
cfg.initial=[cfg.initial(1:6),repmat(.5,1,n-4),repmat(cfg.grfSlope(3),1,n-1)];
cfg.grfPairs=[];
for k=2:n-2
 for j=setdiff(2:n-1,k)
  cfg.grfPairs(end+1,:)=[k,j]; %#ok<AGROW>
 end
end
cfg.shapeConstraintCount=3*(n-1);
validateattributes(cfg.starts,{'numeric'},{'scalar','integer','>=',size(cfg.grfPairs,1)});
end
