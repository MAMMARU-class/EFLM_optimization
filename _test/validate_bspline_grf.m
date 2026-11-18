function validate_bspline_grf()
root=fileparts(fileparts(mfilename('fullpath'))); addpath(genpath(fullfile(root,'common')),fullfile(root,'stage1_point_mass'));
u=linspace(0,1,1001)'; op=bspline_operators(16,u);
assert(max(abs(sum(op.B,2)-1))<1e-12);
assert(min(op.B,[],'all')>=-1e-14);
assert(max(abs(sum(op.D,2)))<1e-10);
assert(max(abs(sum(op.I,2)-u))<1e-12);
assert(max(abs(sum(op.J,2)-u.^2/2))<1e-12);
knots=op.knots; weights=(knots(5:end)-knots(1:end-4))/4;
assert(max(abs(op.I(end,:)-weights))<1e-12);
assert(abs(bspline_absolute_area(ones(16,1),knots)-1)<1e-12);
greville=zeros(16,1);
for i=1:16, greville(i)=mean(knots(i+1:i+3)); end
assert(abs(bspline_absolute_area(greville-.5,knots)-.25)<1e-12);
cfg=optimization_settings(); cfg.layoutVersion=3; cfg.bsplineXN=16; cfg.bsplineZN=11;
cfg.bsplineForceBound=50;
cfg.index.fx=6:19; cfg.index.fz=20:28;
a=[2.2 deg2rad(50) .3 1 .5 linspace(-10,10,14) ones(1,9)/9];
p=robot_parameters(); b=prescribed_bspline_motion(a,p,121,cfg);
assert(max(abs([b.Fx([1 end]);b.Fz([1 end])]))<1e-10);
assert(min(b.Fz)>=-1e-10);
assert(max(abs(b.Fx))<=cfg.bsplineForceBound+1e-10);
assert(max(abs(b.periodic))<1e-10);
assert(abs(b.x(end)-b.x(1)-b.referenceDisplacement)<1e-10);
% Compare analytic derivatives against differences away from endpoints.
du=1e-6; mid=[.13;.42;.78]; q=bspline_operators(16,mid);
bp=bspline_basis(mid+du,knots,3); bm=bspline_basis(mid-du,knots,3);
assert(max(abs((bp-bm)/(2*du)-q.D),[],'all')<1e-6);
fprintf('B-spline basis, quadrature, endpoint, derivative and periodicity checks passed.\n');
end
