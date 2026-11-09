function a=basic_bounded_starts(cfg,basic)
% Box-constrained starts satisfying the three linear normalization equalities.
a=cfg.lower+rand(cfg.startsPerK,numel(cfg.initial)).*(cfg.upper-cfg.lower);
a(1,:)=cfg.initial; a(:,1:3)=repmat(basic,size(a,1),1);
ix=cfg.index; nodes=setdiff(2:cfg.grfXN-1,cfg.grfK);
neg=ix.fx(nodes<cfg.grfK); pos=ix.fx(nodes>cfg.grfK);
for row=1:size(a,1)
 % Feasible endpoint-beta differences implied by the available side masses.
 low=max(sum(cfg.lower(neg)),1-sum(cfg.upper(pos)));
 high=min(sum(cfg.upper(neg)),1-sum(cfg.lower(pos)));
 dlow=max(24*(low-.5),cfg.lower(ix.betaXStart)-cfg.upper(ix.betaXEnd));
 dhigh=min(24*(high-.5),cfg.upper(ix.betaXStart)-cfg.lower(ix.betaXEnd));
 if dlow>dhigh+1e-12, error('EFLM:LocalBox','No normalized horizontal shape in local bounds.'); end
 d=min(dhigh,max(dlow,a(row,ix.betaXStart)-a(row,ix.betaXEnd)));
 blo=max(cfg.lower(ix.betaXEnd),cfg.lower(ix.betaXStart)-d);
 bhi=min(cfg.upper(ix.betaXEnd),cfg.upper(ix.betaXStart)-d);
 bend=min(bhi,max(blo,a(row,ix.betaXEnd)));
 a(row,ix.betaXEnd)=bend; a(row,ix.betaXStart)=bend+d;
 a(row,neg)=project(a(row,neg),cfg.lower(neg),cfg.upper(neg),.5+d/24);
 a(row,pos)=project(a(row,pos),cfg.lower(pos),cfg.upper(pos),.5-d/24);
 a(row,ix.fz)=project(a(row,ix.fz),cfg.lower(ix.fz),cfg.upper(ix.fz),1);
end
end

function x=project(y,lo,hi,total)
if sum(lo)>total+1e-10 || sum(hi)<total-1e-10
 error('EFLM:LocalBox','Normalization cannot be satisfied inside local bounds.');
end
left=min(y-hi)-1; right=max(y-lo)+1;
for it=1:55
 mid=(left+right)/2; x=min(hi,max(lo,y-mid));
 if sum(x)>total, left=mid; else, right=mid; end
end
x=min(hi,max(lo,y-(left+right)/2));
end
