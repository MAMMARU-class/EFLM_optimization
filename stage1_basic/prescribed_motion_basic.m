function b = prescribed_motion_basic(a,p,n,cfg)
% Layout v2: impulse-controlled GRF and first-order stance-time equation.
ix=cfg.index; v=a(ix.vx0); thetaL=a(ix.thetaL); r0=a(ix.r0);
D=a(ix.duty); alpha=a(ix.stanceScale); Ix=a(ix.Ix);
shape=horizontal_impulse_shape(a,cfg); L=2*abs(r0*sin(thetaL));
if ~isfinite(shape.Sx) || shape.Sx<=0 || Ix<0
 error('EFLM:HorizontalImpulse','Nonpositive shape area or negative absolute impulse');
end
meanV=v+Ix/(p.m*shape.Sx)*(shape.S0-shape.CH/(cfg.grfXN-1));
if ~isfinite(meanV) || meanV<=0 || L<=0
 error('EFLM:StanceTime','Mean stance velocity must be positive');
end
TsBase=L/meanV; Ts=alpha*TsBase;
if ~isfinite(Ts) || Ts<=0 || D<=0 || D>=1
 error('EFLM:StanceTime','Invalid stance duration or duty');
end
Tf=Ts*(1/D-1); T=Ts+Tf; dt=Ts/(cfg.grfXN-1);
Ax=Ix/(dt*shape.Sx);
t=Ts*(1-cos(linspace(0,pi,n)'))/2;
grf=hermite_horizontal_impulse(a,Ts,t,cfg,shape);
gz=hermite_vertical_grf(a,Ts,t,cfg);
Az=p.m*p.g*T/gz.impulse; vz0=p.g*(gz.C*gz.dt/gz.S-T/2);
x0=-r0*sin(thetaL); z0=r0*cos(thetaL);
Fx=Ax*grf.f; Fz=Az*gz.f;
vx=v+Ax*grf.integral/p.m; x=x0+v*t+Ax*grf.doubleIntegral/p.m;
vz=vz0+Az*gz.integral/p.m-p.g*t;
z=z0+vz0*t+Az*gz.doubleIntegral/p.m-p.g*t.^2/2;
b=struct('t',t,'x',x,'z',z,'vx',vx,'vz',vz,'Fx',Fx,'Fz',Fz,...
 'dFx',Ax*grf.df,'dFz',Az*gz.df,'Ts',Ts,'Tf',Tf,'T',T,...
 'r0',r0,'TsBase',TsBase,'stanceScale',alpha,'vz0',vz0);
b.flightTime=linspace(0,Tf,max(3,ceil(n*Tf/Ts)))'; u=b.flightTime;
b.flightX=x(end)+vx(end)*u; b.flightZ=z(end)+vz(end)*u-p.g*u.^2/2;
b.flightVx=repmat(vx(end),size(u)); b.flightVz=vz(end)-p.g*u;
b.distance=b.flightX(end)-x0; b.speed=b.distance/T;
b.periodic=[b.flightZ(end)-z0,b.flightVx(end)-v,b.flightVz(end)-vz0];
grf.amplitude=Ax; grf.absoluteImpulseTarget=Ix;
b.horizontalGRF=grf; gz.amplitude=Az; b.verticalGRF=gz;
b.meanStanceVelocity=meanV; b.referenceDisplacement=L;
end
