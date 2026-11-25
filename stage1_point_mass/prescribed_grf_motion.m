function b=prescribed_grf_motion(a,p,n,cfg)
% Layout 4: independent stance time, shared x/z basis, actual Fx controls.
ix=cfg.index; v=a(1); angle=a(2); r0=a(3); Ts=a(4); D=a(5);
Tf=Ts*(1/D-1); T=Ts/D;
cx=[0;a(ix.fx)';0]; cz=[0;a(ix.fz)';0];
endpoint=grf_operators(cfg.curveType,cfg.controlCount,1);
Sz=endpoint.I*cz;
if Sz<=0, error('EFLM:VerticalImpulse','Zero vertical shape area.'); end
Az=p.m*p.g/(D*Sz);
vz0=p.g*(Ts*((endpoint.I-endpoint.J)*cz)/Sz-T/2);
u=(1-cos(linspace(0,pi,n)'))/2; t=Ts*u;
op=grf_operators(cfg.curveType,cfg.controlCount,u);
x0=-r0*sin(angle); z0=r0*cos(angle);
Fx=op.B*cx; Fz=Az*op.B*cz;
vx=v+Ts*op.I*cx/p.m; x=x0+v*t+Ts^2*op.J*cx/p.m;
vz=vz0+Az*Ts*op.I*cz/p.m-p.g*t;
z=z0+vz0*t+Az*Ts^2*op.J*cz/p.m-p.g*t.^2/2;
b=struct('t',t,'x',x,'z',z,'vx',vx,'vz',vz,'Fx',Fx,'Fz',Fz,...
 'dFx',op.D*cx/Ts,'dFz',Az*op.D*cz/Ts,'Ts',Ts,'Tf',Tf,'T',T,'r0',r0,'vz0',vz0);
b.flightTime=linspace(0,Tf,max(3,ceil(n*Tf/Ts)))'; f=b.flightTime;
b.flightX=x(end)+vx(end)*f; b.flightZ=z(end)+vz(end)*f-p.g*f.^2/2;
b.flightVx=repmat(vx(end),size(f)); b.flightVz=vz(end)-p.g*f;
b.distance=b.flightX(end)-x0; b.speed=b.distance/T;
b.periodic=[b.flightZ(end)-z0,b.flightVx(end)-v,b.flightVz(end)-vz0];
capacity=p.frictionCoefficient*Az*cz;
controlConstraints=[cx(2:end-1)-capacity(2:end-1);-cx(2:end-1)-capacity(2:end-1)]/(p.m*p.g);
curveConstraints=zeros(0,1);
if strcmp(cfg.curveType,'hermite')
 curveConstraints=-[hermite_segment_minima(op,Az*cz);...
  hermite_segment_minima(op,capacity-cx);hermite_segment_minima(op,capacity+cx)]/(p.m*p.g);
end
b.horizontalGRF=struct('shapeConstraints',controlConstraints,'normalizedImpulse',endpoint.I*cx/(p.m*p.g));
b.verticalGRF=struct('shapeConstraints',curveConstraints,'normalizationResidual',sum(cz)-1);
b.grf=struct('cx',cx,'cz',cz,'Ax',1,'Az',Az,'kind',cfg.curveType);
end
