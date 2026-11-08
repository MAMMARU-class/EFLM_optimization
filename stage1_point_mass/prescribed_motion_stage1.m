function b = prescribed_motion_stage1(a,p,n,cfg)
% Exact integration of m*xdd=Fx, m*zdd=Fz-m*g; toe fixed at world origin.
A=a(1); alpha=a(2); v=a(3); D=a(4);
r0=a(5);
% General displacement identity, also valid before the impulse constraint holds.
shape=horizontal_grf_shape(a,cfg);
L=2*r0*sin(alpha);
gamma=A*(shape.CH-(cfg.grfN-1)*shape.S)/(p.m*(cfg.grfN-1)^2);
discriminant=v^2-4*gamma*L;
if ~isfinite(discriminant) || discriminant<0 || v<=0 || L<=0
 error('EFLM:StanceTime','No real positive short-branch base stance time');
end
stanceScale=a(6); TsBase=2*L/(v+sqrt(discriminant));
Ts=stanceScale*TsBase;
if ~isfinite(Ts) || Ts<=0
 error('EFLM:StanceTime','Stance time must be finite and positive');
end
Tf=Ts*(1/D-1); T=Ts+Tf;
% Cluster time nodes near touchdown/liftoff to resolve contact constraints.
t=Ts*(1-cos(linspace(0,pi,n)'))/2;
gz=hermite_vertical_grf(a,Ts,t,cfg);
Az=p.m*p.g*T/gz.impulse;
vz0=p.g*(gz.C*gz.dt/gz.S-T/2);
x0=-r0*sin(alpha); z0=r0*cos(alpha);
grf=hermite_horizontal_grf(a,Ts,t,cfg,shape);
Fx=A*grf.f; Fz=Az*gz.f;
vx=v+A*grf.integral/p.m;
x=x0+v*t+A*grf.doubleIntegral/p.m;
vz=vz0+Az*gz.integral/p.m-p.g*t;
z=z0+vz0*t+Az*gz.doubleIntegral/p.m-p.g*t.^2/2;
b=struct('t',t,'x',x,'z',z,'vx',vx,'vz',vz,'Fx',Fx,'Fz',Fz,...
 'dFx',A*grf.df,'dFz',Az*gz.df,...
 'Ts',Ts,'Tf',Tf,'T',T,'r0',r0,'TsBase',TsBase,'stanceScale',stanceScale);
b.flightTime=linspace(0,Tf,max(3,ceil(n*Tf/Ts)))';
u=b.flightTime;
b.flightX=x(end)+vx(end)*u;
b.flightZ=z(end)+vz(end)*u-p.g*u.^2/2;
b.flightVx=repmat(vx(end),size(u)); b.flightVz=vz(end)-p.g*u;
b.distance=b.flightX(end)-x0; b.speed=b.distance/T;
b.periodic=[b.flightZ(end)-z0,b.flightVx(end)-v,b.flightVz(end)-vz0];
b.horizontalGRF=grf;
gz.amplitude=Az; b.verticalGRF=gz; b.vz0=vz0;
b.stanceGamma=gamma; b.stanceDiscriminant=discriminant; b.referenceDisplacement=L;
end
