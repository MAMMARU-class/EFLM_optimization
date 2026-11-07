function b = prescribed_motion_mean_velocity(a,p,n)
% Exact integration of m*xdd=Fx, m*zdd=Fz-m*g; toe fixed at world origin.
A=a(1); alpha=a(2); v=a(3); D=a(4);
r0=a(5);
% Match stance displacement to 2*r0*sin(alpha), including horizontal GRF:
% A*Ts^2 - Bx*Ts + Cx = 0. Choose the shorter, forward-moving root.
Bx=2*pi*p.m*v; Cx=4*pi*p.m*r0*sin(alpha);
discriminant=Bx^2-4*A*Cx;
if ~isfinite(discriminant) || discriminant<0
 error('EFLM:StanceTime','No real stance time for the prescribed horizontal force and displacement');
end
% Rationalized root avoids cancellation and also covers A=0 (Ts=Cx/Bx).
Ts=2*Cx/(Bx+sqrt(discriminant));
if ~isfinite(Ts) || Ts<=0
 error('EFLM:StanceTime','Stance time must be finite and positive');
end
Tf=Ts*(1/D-1); T=Ts+Tf;
% Cluster time nodes near touchdown/liftoff to resolve contact constraints.
t=Ts*(1-cos(linspace(0,pi,n)'))/2; w=pi/Ts; B=pi*p.m*p.g/(2*D);
x0=-r0*sin(alpha); z0=r0*cos(alpha); vz0=-p.g*Tf/2;
Fx=-A*sin(2*w*t); Fz=B*sin(w*t);
vx=v+A/(2*w*p.m)*(cos(2*w*t)-1);
x=x0+v*t+A/(2*w*p.m)*(sin(2*w*t)/(2*w)-t);
vz=vz0+B/(w*p.m)*(1-cos(w*t))-p.g*t;
z=z0+vz0*t+B/(w*p.m)*(t-sin(w*t)/w)-p.g*t.^2/2;
b=struct('t',t,'x',x,'z',z,'vx',vx,'vz',vz,'Fx',Fx,'Fz',Fz,...
 'dFx',-2*w*A*cos(2*w*t),'dFz',w*B*cos(w*t),...
 'Ts',Ts,'Tf',Tf,'T',T,'r0',r0);
b.flightTime=linspace(0,Tf,max(3,ceil(n*Tf/Ts)))';
u=b.flightTime;
b.flightX=x(end)+vx(end)*u;
b.flightZ=z(end)+vz(end)*u-p.g*u.^2/2;
b.flightVx=repmat(vx(end),size(u)); b.flightVz=vz(end)-p.g*u;
b.distance=b.flightX(end)-x0; b.speed=b.distance/T;
b.periodic=[b.flightZ(end)-z0,b.flightVx(end)-v,b.flightVz(end)-vz0];
end
