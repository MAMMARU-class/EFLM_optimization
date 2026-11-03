function tab = build_geometry_cache(p)
% One-dimensional geometry table, not an optimization grid search.
k=linspace(deg2rad(12),deg2rad(148),4097)'; h=zeros(size(k));
[Y,qa]=eflm_geometry(h,k,0*k,p);
assert(isreal(Y)&&all(isfinite(Y),'all'),'Invalid reference geometry');
epsc=1e-20;
Ys=imag(eflm_geometry(h,k,1i*epsc+0*k,p))/epsc;
l=sqrt(sum(Y.^2,2)); beta=atan2(-Y(:,2),Y(:,1));
f=-sum(Y.*Ys,2)./l;
g=-(Y(:,2).*Ys(:,1)-Y(:,1).*Ys(:,2))./l.^2;
names={'l','beta','f','g','qa'}; vals={l,beta,f,g,qa};
for i=1:numel(names)
 pp=spline(k,vals{i}); tab.(names{i})=pp;
 [br,co,~,ord,dim]=unmkpp(pp);
 tab.([names{i} 'd'])=mkpp(br,co(:,1:ord-1).*(ord-1:-1:1),dim);
end
assert(all(diff(l)<0),'Reference length must be monotone');
tab.inverse= pchip(flipud(l),flipud(k));
tab.kRange=k([1 end]); tab.lRange=[min(l) max(l)];
tab.samples=[k,l,beta,f,g,qa];
end
