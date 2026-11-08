function validate_mean_velocity_stance()
root=fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root,'common')),fullfile(root,'stage1_point_mass'));
p=robot_parameters(); cfg=configure_horizontal_grf(optimization_settings());
a=cfg.initial; a(7:9)=[.5 .75 .75]; a(end)=-.2; a(6)=1;
for imbalance=[0 .05]
 q=a; q(7)=q(7)+imbalance;
 b=prescribed_motion_stage1(q,p,20001,cfg);
 dt=b.Ts/(cfg.grfN-1); h=b.horizontalGRF;
 assert(abs(trapz(b.t,b.t.*h.f)-h.CH*dt^2)<1e-8);
 assert(abs(trapz(b.t,h.f)-h.S*dt)<1e-8);
 assert(abs(b.x(end)-b.x(1)-2*q(5)*sin(q(2)))<1e-12);
end
q=a; q(1)=0; b=prescribed_motion_stage1(q,p,101,cfg);
assert(abs(b.Ts-2*q(5)*sin(q(2))/q(3))<1e-14);
% Fixed beta must preserve shape when physical duration changes.
u=linspace(0,1,1001)'; h1=hermite_horizontal_grf(a,.2,.2*u,cfg);
h2=hermite_horizontal_grf(a,.6,.6*u,cfg);
assert(max(abs(h1.f-h2.f))<1e-12);
assert(max(abs(h1.slopes-3*h2.slopes))<1e-10);
% Negative discriminants are rejected before geometry evaluation.
q=a; q(1)=1e6;
try
 prescribed_motion_stage1(q,p,101,cfg);
 error('Test:MissingError','Expected a negative-discriminant rejection');
catch ME
 assert(strcmp(ME.identifier,'EFLM:StanceTime'));
end
fprintf('Mean-velocity stance: moments, nonzero impulse, time scaling and invalid roots passed.\n');
end
