function cfg = configure_horizontal_grf(cfg)
% n is fixed for one run; k and j are categorical choices in the broad stage.
n=cfg.grfN;
validateattributes(n,{'numeric'},{'scalar','integer','>=',4});
validateattributes(cfg.grfSlope,{'numeric'},{'real','finite','size',[1 3]});
endpointMargin=1e-4;
if cfg.grfSlope(1)>=-endpointMargin || cfg.grfSlope(2)<=cfg.grfSlope(1)
 error('EFLM:GRFSlope','grfSlope must permit negative endpoint slopes in s^-1.');
end
if cfg.lower(1)<0, error('EFLM:GRFAmplitude','AmpH must be nonnegative.'); end
cfg.variableNames={'AmpH','thetaL','v','Duty','r0','stanceScale'};
for i=1:n-4, cfg.variableNames{end+1}=sprintf('grfRatio%d',i); end
cfg.variableNames=[cfg.variableNames,{'grfSlopeStart','grfSlopeEnd'}];
endpointUpper=min(cfg.grfSlope(2),-endpointMargin);
endpointSeed=abs(cfg.grfSlope(3));
if endpointSeed==0, endpointSeed=1; end
endpointSeed=min(max(endpointSeed,-endpointUpper),-cfg.grfSlope(1));
cfg.lower=[cfg.lower(1:6),repmat(1e-4,1,n-4),cfg.grfSlope(1),cfg.grfSlope(1)];
cfg.upper=[cfg.upper(1:6),repmat(1-1e-4,1,n-4),endpointUpper,endpointUpper];
cfg.initial=[cfg.initial(1:6),repmat(.5,1,n-4),-endpointSeed,-endpointSeed];
cfg.grfPairs=[];
for k=2:n-2
 for j=setdiff(2:n-1,k)
  cfg.grfPairs(end+1,:)=[k,j]; %#ok<AGROW>
 end
end
cfg.grfK=ceil(n/2); choices=setdiff(2:n-1,cfg.grfK); cfg.grfJ=choices(1);
cfg.shapeConstraintCount=3*(n-1);
validateattributes(cfg.starts,{'numeric'},{'scalar','integer','>=',size(cfg.grfPairs,1)});
end
