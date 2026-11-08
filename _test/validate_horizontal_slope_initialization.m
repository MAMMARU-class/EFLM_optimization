function validate_horizontal_slope_initialization()
root=fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root,'common')));
rng(17);
for n=[4 7 21]
 cfg=optimization_settings(); cfg.grfN=n; cfg.starts=max(16,(n-3)^2);
 cfg.grfSlope=[-40 40 0]; cfg=configure_horizontal_grf(cfg);
 for pair=unique(round(linspace(1,size(cfg.grfPairs,1),min(12,size(cfg.grfPairs,1)))))
  cfg.grfK=cfg.grfPairs(pair,1); cfg.grfJ=cfg.grfPairs(pair,2);
  for narrow=[false true]
   lower=cfg.lower; upper=cfg.upper;
   if narrow
    width=upper-lower; lower=lower+.3*width; upper=upper-.2*width;
   end
   a=lower+rand(10,numel(lower)).*(upper-lower);
   [seed,check]=initialize_horizontal_grf_slopes(a,cfg,lower,upper);
   columns=(n+3):numel(lower);
   assert(isequal(a(:,1:n+2),seed(:,1:n+2)),'Non-slope variables changed');
   assert(all(seed>=lower-1e-12 & seed<=upper+1e-12,'all'));
   for row=1:size(a,1)
    Ts=a(row,6)*2*a(row,5)*sin(a(row,2))/a(row,3);
    h=hermite_horizontal_grf(seed(row,:),Ts,linspace(0,Ts,15)',cfg);
    expected=zeros(1,n);
    expected(2:n-1)=(h.values(3:n)'-h.values(1:n-2)')/(2*Ts/(n-1));
    nodes=setdiff(1:n,cfg.grfJ);
    expected(nodes)=min(upper(columns),max(lower(columns),expected(nodes)));
    expected(cfg.grfJ)=0;
    assert(max(abs(h.slopes'-expected))<1e-10);
    assert(abs(check.shapeViolation(row)-max([0;h.shapeConstraints]))<1e-10);
    assert(abs(check.normalizedImpulse(row)-h.normalizedImpulse)<1e-10);
   end
  end
 end
end
% Invalid duration must not abort all other MultiStart candidates.
invalid=cfg.initial; invalid(3)=0;
[seed,check]=initialize_horizontal_grf_slopes(invalid,cfg);
assert(isequal(seed,invalid) && ~check.feasible);
fprintf('Central-difference initialization: bounds, zero endpoints and peak, unchanged ratios, and analytic checks passed.\n');
end
