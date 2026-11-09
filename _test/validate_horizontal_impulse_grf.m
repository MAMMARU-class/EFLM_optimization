function validate_horizontal_impulse_grf()
% Independent quadrature checks, including candidates off the equalities.
root=fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root,'common')),fullfile(root,'stage1_point_mass'));
p=robot_parameters(); cfg=configure_impulse_grf(optimization_settings());
cfg.grfK=8; a=cfg.initial;
a(cfg.index.betaXStart)=-.1; a(cfg.index.betaXEnd)=-.2;
a=seed_horizontal_impulse(a,cfg,false);
for imbalance=[0 .03]
 q=a; q(cfg.index.fx(1))=q(cfg.index.fx(1))+imbalance;
 for alpha=[.7 1 1.4]
  q(cfg.index.stanceScale)=alpha;
  b=prescribed_motion_stage1(q,p,30001,cfg); h=b.horizontalGRF;
  dt=b.Ts/(cfg.grfXN-1);
  assert(abs(trapz(b.t,h.f)-dt*h.S0)<1e-8);
  assert(abs(trapz(b.t,b.t.*h.f)-dt^2*h.CH)<1e-8);
  assert(abs(b.x(end)-b.x(1)-alpha*b.referenceDisplacement)<1e-12);
  assert(max(abs(b.periodic([1 3])))<1e-12);
  assert(abs(b.vx(end)-b.vx(1)-h.amplitude*dt*h.S0/p.m)<1e-12);
  if imbalance==0
   assert(max(h.shapeConstraints)<1e-10);
   assert(abs(trapz(b.t,abs(b.Fx))-q(cfg.index.Ix))<1e-7);
  end
 end
end
u=linspace(0,1,1001)';
h1=hermite_horizontal_impulse(a,.2,.2*u,cfg);
h2=hermite_horizontal_impulse(a,.6,.6*u,cfg);
for i=1:cfg.grfXN-2
 left=h1.coefficients(i,:); right=h1.coefficients(i+1,:);
 assert(abs(polyval(left,1)-polyval(right,0))<1e-12);
 assert(abs(polyval(polyder(left),1)-polyval(polyder(right),0))<1e-12);
end
assert(max(abs(h1.beta(2:end-1)-(h1.values(3:end)-h1.values(1:end-2))/2))<1e-12);
assert(max(abs(h1.f-h2.f))<1e-12);
assert(max(abs(h1.slopes-3*h2.slopes))<1e-10);
for k=cfg.grfKValues
 cfg.grfK=k; q=seed_horizontal_impulse(a,cfg,true);
 h=horizontal_impulse_shape(q,cfg);
 assert(abs(h.S0)<1e-12 && abs(h.normalizationResidual)<1e-12);
end
fprintf('Impulse GRF: quadrature, off-equality travel, periodicity, scaling and all-k seeds passed.\n');
end
