function tab = build_eflm5_cache(p)
% Zero-extension assembly seeds only. No table approximation in the equations.
k=linspace(deg2rad(12),deg2rad(148),4097)';
[Y,a]=eflm_geometry(zeros(size(k)),k,zeros(size(k)),p);
R=sqrt(sum(Y.^2,2));
assert(all(diff(R)<0),'Invalid reference assembly');
tab.inverse=pchip(flipud(R),flipud(k));
tab.ankle=pchip(k,a); tab.kRange=k([1,end]); tab.lRange=[min(R),max(R)];
tab.role='Initial guesses only; five-coordinate equations evaluated directly';
end
