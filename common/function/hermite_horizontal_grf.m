function h = hermite_horizontal_grf(a,Ts,t,cfg,shapeData)
% Cubics in local u=(t-ti)/dt. Integrals and extrema are analytic.
n=cfg.grfN; k=cfg.grfK; j=cfg.grfJ; dt=Ts/(n-1);
if nargin<5, shapeData=horizontal_grf_shape(a,cfg); end
value=shapeData.values; slope=shapeData.beta/dt;
ratioNodes=shapeData.ratioNodes; slopeNodes=[1 n]; coef=shapeData.coefficients;
area=dt*(coef(:,1)/4+coef(:,2)/3+coef(:,3)/2+coef(:,4));
first=[0;cumsum(area)];
second=[0;cumsum(dt*first(1:end-1)+dt^2*(coef(:,1)/20+coef(:,2)/12+coef(:,3)/6+coef(:,4)/2))];
segment=min(n-1,max(1,floor(t(:)/dt)+1)); u=t(:)/dt-(segment-1);
c=coef(segment,:);
h.f=((c(:,1).*u+c(:,2)).*u+c(:,3)).*u+c(:,4);
h.df=(3*c(:,1).*u.^2+2*c(:,2).*u+c(:,3))/dt;
h.integral=first(segment)+dt*(c(:,1).*u.^4/4+c(:,2).*u.^3/3+c(:,3).*u.^2/2+c(:,4).*u);
h.doubleIntegral=second(segment)+first(segment).*dt.*u+dt^2*(...
 c(:,1).*u.^5/20+c(:,2).*u.^4/12+c(:,3).*u.^3/6+c(:,4).*u.^2/2);
% Check the full cubic on each segment, including all interior stationary points.
shape=zeros(n-1,3);
for i=1:n-1
 c=coef(i,:); rootsU=[];
 if abs(c(1))>1e-14
  discriminant=4*c(2)^2-12*c(1)*c(3);
  if discriminant>=0
   rootsU=[-2*c(2)-sqrt(discriminant),-2*c(2)+sqrt(discriminant)]/(6*c(1));
  end
 elseif abs(c(2))>1e-14
  rootsU=-c(3)/(2*c(2));
 end
 rootsU=rootsU(rootsU>0 & rootsU<1);
 extrema=polyval(c,rootsU);
 direction=1; if i<k, direction=-1; end
 shape(i,1)=max([-direction*value(i:i+1),1e-4-direction*extrema]);
 shape(i,2)=max(abs([value(i:i+1),extrema]))-1;
 % Reject intervals whose force remains identically zero.
 shape(i,3)=1e-4-direction*area(i)/dt;
end
h.shapeConstraints=shape(:);
h.impulse=first(end); h.normalizedImpulse=h.impulse/Ts;
h.knots=linspace(0,Ts,n)'; h.values=value'; h.slopes=slope';
h.ratioNodes=ratioNodes; h.slopeNodes=slopeNodes; h.coefficients=coef;
h.k=k; h.j=j;
h.beta=shapeData.beta'; h.S=shapeData.S; h.CH=shapeData.CH;
end
