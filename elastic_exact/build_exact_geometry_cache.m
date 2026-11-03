function tab = build_exact_geometry_cache(p)
% This table seeds Newton only; accepted states use direct exact geometry.
k=linspace(deg2rad(12),deg2rad(148),4097)';
d=exact_polar_geometry(k,zeros(size(k)),p);
assert(all(isfinite(d.R)) && all(diff(d.R)<0),'Invalid EFLM reference branch');
tab.inverse=pchip(flipud(d.R),flipud(k));
tab.kRange=k([1 end]); tab.lRange=[min(d.R),max(d.R)];
tab.samples=[k,d.R]; tab.role='Newton initial guess only';
end
