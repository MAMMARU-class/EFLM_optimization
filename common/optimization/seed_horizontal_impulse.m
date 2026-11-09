function a = seed_horizontal_impulse(a,cfg,randomize)
% Normalize magnitudes on each side; candidate rows may have independent beta.
ix=cfg.index; nodes=setdiff(2:cfg.grfXN-1,cfg.grfK);
neg=nodes<cfg.grfK; pos=~neg;
if randomize
 weights=-log(max(realmin,rand(size(a,1),numel(nodes))));
else
 weights=max(realmin,a(:,ix.fx)-cfg.lower(ix.fx));
end
% P+N=1 and P-N+(beta_1-beta_n)/12=0.
negativeMass=.5+(a(:,ix.betaXStart)-a(:,ix.betaXEnd))/24;
negativeMass=min(1,max(0,negativeMass));
lower=cfg.lower(ix.fx); upper=cfg.upper(ix.fx);
for side=1:2
 mask=neg; target=negativeMass;
 if side==2, mask=pos; target=1-negativeMass; end
 floorMass=sum(lower(mask)); target=max(floorMass,target);
 values=lower(mask)+(target-floorMass).*weights(:,mask)./sum(weights(:,mask),2);
 a(:,ix.fx(mask))=min(upper(mask),values);
end
end
