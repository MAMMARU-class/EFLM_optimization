function area=bspline_absolute_area(c,knots)
% Postprocessing only: exact absolute integral, split at real cubic roots.
breaks=unique(knots); u=[0;1/3;2/3;1]; V=[u.^3 u.^2 u ones(4,1)]; area=0;
for j=1:numel(breaks)-1
 width=breaks(j+1)-breaks(j);
 coef=V\(bspline_basis(breaks(j)+width*u,knots,3)*c(:));
 rootsU=roots(coef'); rootsU=real(rootsU(abs(imag(rootsU))<1e-10));
 edges=unique([0;rootsU(rootsU>0 & rootsU<1);1]);
 primitive=[coef(1)/4 coef(2)/3 coef(3)/2 coef(4) 0];
 area=area+width*sum(abs(diff(polyval(primitive,edges))));
end
end
