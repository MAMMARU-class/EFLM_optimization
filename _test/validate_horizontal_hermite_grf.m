function validate_horizontal_hermite_grf()
% Independent quadrature, endpoint/derivative continuity, and evaluator wiring.
root=fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root,'common')),fullfile(root,'stage1_point_mass'));
cfg=configure_horizontal_grf(optimization_settings()); p=robot_parameters();
a=cfg.initial; a(7:9)=[.5 .75 .75]; Ts=.24;
t=linspace(0,Ts,10001)'; h=hermite_horizontal_grf(a,Ts,t,cfg);
assert(abs(h.impulse)<1e-14);
assert(max(h.shapeConstraints)<=1e-12);
assert(max(abs(h.f([1 end])))<1e-12);
for i=1:cfg.grfN-2
 dt=Ts/(cfg.grfN-1); left=h.coefficients(i,:); right=h.coefficients(i+1,:);
 assert(abs(polyval(left,1)-polyval(right,0))<1e-12);
 assert(abs(polyval(polyder(left),1)-polyval(polyder(right),0))/dt<1e-10);
end
% Interior slopes are reconstructed from neighboring values at every evaluation.
expected=(h.values(3:end)'-h.values(1:end-2)')/(2*dt);
expected(cfg.grfJ-1)=0;
assert(max(abs(h.slopes(2:end-1)'-expected))<1e-12);
assert(numel(a)==cfg.grfN+4 && all(a(end-1:end)<0));
% Endpoint slope does change the integral by dt^2*delta_b/12.
q=a; q(end)=q(end)-3;
h3=hermite_horizontal_grf(q,Ts,t,cfg);
assert(abs(h3.impulse-h.impulse-3*dt^2/12)<1e-13);
assert(abs(trapz(t,h3.f)-h3.impulse)<1e-8);
assert(max(abs(cumtrapz(t,h3.f)-h3.integral))<1e-8);
assert(max(abs(cumtrapz(t,h3.integral)-h3.doubleIntegral))<1e-8);
% Dense sampling independently checks that extrema bounds find overshoots.
q=a; q(end)=-1000;
bad=hermite_horizontal_grf(q,Ts,t,cfg);
assert(max(abs(bad.f))>1 && max(bad.shapeConstraints)>0);
tab=build_exact_geometry_cache(p);
r=sim_cycle_stage1_point_mass(a,p,tab,cfg,121);
assert(numel(r.constraints)==16*121+cfg.shapeConstraintCount+3);
assert(numel(r.constraintEnvelope)==19+cfg.shapeConstraintCount);
assert(abs(r.equalityConstraints)<1e-12);
assert(abs(r.body.periodic(2))<1e-12);
assert(max(abs(r.exactToeResidual),[],'all')<1e-10);
% Configurable n also covers the n=5 boundary (one free ratio variable).
for points=[5 8]
 other=optimization_settings(); other.grfN=points; other.starts=max(16,(points-4)*(points-3));
 other=configure_horizontal_grf(other);
 assert(numel(other.initial)==points+4);
 for pair=1:size(other.grfPairs,1)
  other.grfK=other.grfPairs(pair,1); other.grfJ=other.grfPairs(pair,2);
  z=hermite_horizontal_grf(other.initial,Ts,t,other);
  assert(numel(z.shapeConstraints)==3*(points-1));
  assert(abs(trapz(t,z.f)-z.impulse)<1e-8);
  assert(z.values(other.grfK)==0 && z.slopes(other.grfJ)==0);
  assert(all(z.slopes([1 end])<0));
 end
end
fprintf('Hermite GRF: analytic integrals, continuity, extrema, and model wiring passed.\n');
end
