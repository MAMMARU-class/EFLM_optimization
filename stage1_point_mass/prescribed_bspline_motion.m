function b=prescribed_bspline_motion(a,p,n,cfg)
ix=cfg.index; v=a(1); r0=a(3); angle=a(2); alpha=a(4); D=a(5);
cx=[0;a(ix.fx)';0]; cz=[0;a(ix.fz)';0];
endX=bspline_operators(cfg.bsplineXN,1); endZ=bspline_operators(cfg.bsplineZN,1);
Q=endX.J*cx/p.m; L=2*abs(r0*sin(angle)); disc=v*v+4*Q*L;
if disc<=0, error('EFLM:StanceTime','No positive increasing-displacement base-time root.'); end
TsBase=2*L/(v+sqrt(disc)); Ts=alpha*TsBase; Tf=Ts*(1/D-1); T=Ts+Tf;
if Ts<=0 || Tf<=0, error('EFLM:StanceTime','Nonpositive duration.'); end
u=(1-cos(linspace(0,pi,n)'))/2; t=Ts*u;
hx=bspline_operators(cfg.bsplineXN,u); hz=bspline_operators(cfg.bsplineZN,u);
Sz=endZ.I*cz;
if Sz<=0, error('EFLM:VerticalImpulse','Zero vertical shape area.'); end
Az=p.m*p.g*T/(Ts*Sz); Cz=(endZ.I-endZ.J)*cz;
vz0=p.g*(Ts*Cz/Sz-T/2); x0=-r0*sin(angle); z0=r0*cos(angle);
Fx=hx.B*cx; Fz=Az*hz.B*cz;
vx=v+Ts*hx.I*cx/p.m; x=x0+v*t+Ts^2*hx.J*cx/p.m;
vz=vz0+Az*Ts*hz.I*cz/p.m-p.g*t;
z=z0+vz0*t+Az*Ts^2*hz.J*cz/p.m-p.g*t.^2/2;
b=struct('t',t,'x',x,'z',z,'vx',vx,'vz',vz,'Fx',Fx,'Fz',Fz,...
 'dFx',hx.D*cx/Ts,'dFz',Az*hz.D*cz/Ts,'Ts',Ts,'Tf',Tf,'T',T,...
 'r0',r0,'TsBase',TsBase,'stanceScale',alpha,'vz0',vz0);
b.flightTime=linspace(0,Tf,max(3,ceil(n*Tf/Ts)))'; f=b.flightTime;
b.flightX=x(end)+vx(end)*f; b.flightZ=z(end)+vz(end)*f-p.g*f.^2/2;
b.flightVx=repmat(vx(end),size(f)); b.flightVz=vz(end)-p.g*f;
b.distance=b.flightX(end)-x0; b.speed=b.distance/T;
b.periodic=[b.flightZ(end)-z0,b.flightVx(end)-v,b.flightVz(end)-vz0];
b.horizontalGRF=struct('shapeConstraints',zeros(0,1),'normalizedImpulse',endX.I*cx/cfg.bsplineForceBound);
b.verticalGRF=struct('shapeConstraints',zeros(0,1),'normalizationResidual',sum(cz)-1);
b.bspline=struct('cx',cx,'cz',cz,'Ax',1,'Az',Az,'knotsX',hx.knots,'knotsZ',hz.knots);
b.referenceDisplacement=L;
end
