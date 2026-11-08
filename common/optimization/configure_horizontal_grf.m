function cfg = configure_horizontal_grf(cfg)
% n is fixed for one run; k and j are categorical choices in the broad stage.
n=cfg.grfN;
validateattributes(n,{'numeric'},{'scalar','integer','>=',5});
endpointBounds=cfg.grfBetaEndpoint;
validateattributes(endpointBounds,{'numeric'},{'real','finite','size',[1 3]},mfilename,'grfBetaEndpoint');
if endpointBounds(1)>=endpointBounds(2) || endpointBounds(2)>0 || ...
   endpointBounds(3)<endpointBounds(1) || endpointBounds(3)>endpointBounds(2)
 error('EFLM:GRFSlope','grfBetaEndpoint must be [lower, upper, initial] with lower < upper <= 0.');
end
if cfg.lower(1)<0, error('EFLM:GRFAmplitude','AmpH must be nonnegative.'); end
cfg.variableNames={'AmpH','thetaL','v','Duty','r0','stanceScale'};
for i=1:n-4, cfg.variableNames{end+1}=sprintf('grfRatio%d',i); end
cfg.variableNames=[cfg.variableNames,{'grfBetaEndpoint'}];
cfg.lower=[cfg.lower(1:6),repmat(1e-4,1,n-4),endpointBounds(1)];
cfg.upper=[cfg.upper(1:6),repmat(1-1e-4,1,n-4),endpointBounds(2)];
cfg.initial=[cfg.initial(1:6),repmat(.5,1,n-4),endpointBounds(3)];
validateattributes(cfg.restrictGrfPairs,{'logical'},{'scalar'},mfilename,'restrictGrfPairs');
if cfg.restrictGrfPairs
 pairs=cfg.grfPairSelection;
 validateattributes(pairs,{'numeric'},{'2d','ncols',2,'nonempty','real','finite','integer'},mfilename,'grfPairSelection');
 if any(pairs(:,1)<3 | pairs(:,1)>n-2 | pairs(:,2)<2 | pairs(:,2)>n-1 | pairs(:,1)==pairs(:,2))
  error('EFLM:GRFPairs','Each [k,j] must satisfy 3<=k<=n-2, 2<=j<=n-1 and k~=j.');
 end
 cfg.grfPairs=unique(pairs,'rows','stable');
 cfg.grfK=cfg.grfPairs(1,1); cfg.grfJ=cfg.grfPairs(1,2);
else
 cfg.grfPairs=[];
 for k=3:n-2
  for j=setdiff(2:n-1,k)
   cfg.grfPairs(end+1,:)=[k,j]; %#ok<AGROW>
  end
 end
 cfg.grfK=ceil(n/2); choices=setdiff(2:n-1,cfg.grfK); cfg.grfJ=choices(1);
end
cfg.shapeConstraintCount=3*(n-1);
validateattributes(cfg.starts,{'numeric'},{'scalar','integer','>=',size(cfg.grfPairs,1)});
end
