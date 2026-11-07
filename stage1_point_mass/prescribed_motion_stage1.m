function b = prescribed_motion_stage1(a,p,n,cfg)
% Exact integration of m*xdd=Fx, m*zdd=Fz-m*g; toe fixed at world origin.
A=a(1); alpha=a(2); v=a(3); D=a(4);
r0=a(5);
% Initial-velocity geometric time, scaled by the sixth decision variable.
stanceScale=a(6); TsBase=2*r0*sin(alpha)/v;
Ts=stanceScale*TsBase;
if ~isfinite(Ts) || Ts<=0
 error('EFLM:StanceTime','Stance time must be finite and positive');
end
Tf=Ts*(1/D-1); T=Ts+Tf;
% Cluster time nodes near touchdown/liftoff to resolve contact constraints.
t=Ts*(1-cos(linspace(0,pi,n)'))/2; w=pi/Ts; B=pi*p.m*p.g/(2*D);
x0=-r0*sin(alpha); z0=r0*cos(alpha); vz0=-p.g*Tf/2;
grf=hermite_horizontal_grf(a,Ts,t,cfg);
Fx=A*grf.f; Fz=B*sin(w*t);
vx=v+A*grf.integral/p.m;
x=x0+v*t+A*grf.doubleIntegral/p.m;
vz=vz0+B/(w*p.m)*(1-cos(w*t))-p.g*t;
z=z0+vz0*t+B/(w*p.m)*(t-sin(w*t)/w)-p.g*t.^2/2;
b=struct('t',t,'x',x,'z',z,'vx',vx,'vz',vz,'Fx',Fx,'Fz',Fz,...
 'dFx',A*grf.df,'dFz',w*B*cos(w*t),...
 'Ts',Ts,'Tf',Tf,'T',T,'r0',r0,'TsBase',TsBase,'stanceScale',stanceScale);
b.flightTime=linspace(0,Tf,max(3,ceil(n*Tf/Ts)))';
u=b.flightTime;
b.flightX=x(end)+vx(end)*u;
b.flightZ=z(end)+vz(end)*u-p.g*u.^2/2;
b.flightVx=repmat(vx(end),size(u)); b.flightVz=vz(end)-p.g*u;
b.distance=b.flightX(end)-x0; b.speed=b.distance/T;
b.periodic=[b.flightZ(end)-z0,b.flightVx(end)-v,b.flightVz(end)-vz0];
b.horizontalGRF=grf;
end
