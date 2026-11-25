function points=grf_start_points(stream,count,cfg,problem,isBroad,isFirst)
% Feasible time, z-simplex and Fx cone/zero-impulse starts. Physical Fx stays
% a direct optimization variable; this projection is only start generation.
p=robot_parameters(); lo=cfg.lower; width=cfg.upper-lo;
lb=lo+problem.lb.*width; ub=lo+problem.ub.*width;
op=grf_operators(cfg.curveType,cfg.controlCount,1); w=op.I(2:end-1);
points=zeros(count,numel(lo)); seed=lo+problem.x0.*width;
for row=1:count
 accepted=false;
 for attempt=1:100
  a=lb+rand(stream,1,numel(lo)).*(ub-lb);
  minTs=max(lb(4),cfg.minGaitDuration*lb(5));
  if minTs>ub(4), error('EFLM:Bounds','No feasible duration in start box.'); end
  a(4)=minTs+rand(stream)*(ub(4)-minTs);
  maxD=min(ub(5),a(4)/cfg.minGaitDuration);
  a(5)=lb(5)+rand(stream)*(maxD-lb(5));
  iz=cfg.index.fz; ix=cfg.index.fx;
  if isBroad
   z=a(iz)/sum(a(iz));
  else
   z=projectBox(a(iz),lb(iz),ub(iz),ones(size(w)),1);
  end
  capacity=p.frictionCoefficient*p.m*p.g*z/(a(5)*(w*z'));
  lowerFx=max(lb(ix),-capacity); upperFx=min(ub(ix),capacity);
  if any(lowerFx>upperFx) || w*lowerFx'>1e-10 || w*upperFx'<-1e-10, continue; end
  trial=lowerFx+rand(stream,1,numel(ix)).*(upperFx-lowerFx);
  fx=projectBox(trial,lowerFx,upperFx,w,0);
  a(iz)=z; a(ix)=fx; accepted=true; break;
 end
 if ~accepted
  % The incumbent is a guaranteed fallback only if it satisfies the new box.
  a=seed; b=prescribed_grf_motion(a,p,3,cfg);
  if any(a<lb-1e-9 | a>ub+1e-9) || a(4)/a(5)<cfg.minGaitDuration-1e-9 || ...
    max(b.horizontalGRF.shapeConstraints)>1e-8 || abs(b.horizontalGRF.normalizedImpulse)>1e-8
   error('EFLM:StartGeneration','Cannot generate a cone-feasible narrow start.');
  end
 end
 points(row,:)=(a-lo)./width;
end
if isFirst
 if isBroad, points(1,:)=(cfg.initial-lo)./width;
 else, points(1,:)=problem.x0; end
end
end

function x=projectBox(v,lb,ub,w,target)
% Euclidean projection onto w*x'=target with box bounds (positive w).
if w*lb'>target+1e-10 || w*ub'<target-1e-10
 error('EFLM:StartGeneration','Inconsistent equality and box bounds.');
end
left=min((v-ub)./w)-1; right=max((v-lb)./w)+1;
for iteration=1:60
 middle=(left+right)/2; x=min(ub,max(lb,v-middle*w));
 if w*x'>target, left=middle; else, right=middle; end
end
x=min(ub,max(lb,v-(left+right)/2*w));
end
