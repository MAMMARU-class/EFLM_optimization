function h = hermite_vertical_grf(a,Ts,t,cfg)
% Analytic cubic evaluation, integrals, and full-interval nonnegativity.
n=cfg.grfZN; v=a(cfg.grfZIndices); dt=Ts/(n-1);
f=[0,v(1:n-2),0]; beta=[v(end-1),zeros(1,n-2),v(end)];
beta(2:n-1)=(f(3:n)-f(1:n-2))/2;
y0=f(1:end-1); y1=f(2:end); b0=beta(1:end-1); b1=beta(2:end);
coef=[(2*y0-2*y1+b0+b1)',(-3*y0+3*y1-2*b0-b1)',b0',y0'];
% Use the actual sum during infeasible SQP trials as well.
S=sum(f)+(beta(1)-beta(end))/12;
C=sum((1:n-2).*f(2:end-1))+beta(1)/30+sum(beta(2:end-1))/15-(5*n-7)*beta(end)/60;
if ~isfinite(S) || S<=0, error('EFLM:VerticalImpulse','Nonpositive vertical shape integral'); end
area=dt*(coef(:,1)/4+coef(:,2)/3+coef(:,3)/2+coef(:,4));
first=[0;cumsum(area)];
second=[0;cumsum(dt*first(1:end-1)+dt^2*(coef(:,1)/20+coef(:,2)/12+coef(:,3)/6+coef(:,4)/2))];
segment=min(n-1,max(1,floor(t(:)/dt)+1)); u=t(:)/dt-(segment-1); c=coef(segment,:);
h.f=((c(:,1).*u+c(:,2)).*u+c(:,3)).*u+c(:,4);
h.df=(3*c(:,1).*u.^2+2*c(:,2).*u+c(:,3))/dt;
h.integral=first(segment)+dt*(c(:,1).*u.^4/4+c(:,2).*u.^3/3+c(:,3).*u.^2/2+c(:,4).*u);
h.doubleIntegral=second(segment)+first(segment).*dt.*u+dt^2*(c(:,1).*u.^5/20+c(:,2).*u.^4/12+c(:,3).*u.^3/6+c(:,4).*u.^2/2);
h.shapeConstraints=zeros(n-1,1);
for i=1:n-1
 c=coef(i,:); rootsU=[];
 if abs(c(1))>1e-14
  disc=4*c(2)^2-12*c(1)*c(3);
  if disc>=0, rootsU=[-2*c(2)-sqrt(disc),-2*c(2)+sqrt(disc)]/(6*c(1)); end
 elseif abs(c(2))>1e-14
  rootsU=-c(3)/(2*c(2));
 end
 rootsU=rootsU(rootsU>0 & rootsU<1);
 h.shapeConstraints(i)=-min([f(i:i+1),polyval(c,rootsU)]);
end
h.values=f'; h.beta=beta'; h.slopes=beta'/dt;
h.knots=linspace(0,Ts,n)'; h.coefficients=coef;
h.S=S; h.C=C; h.dt=dt; h.impulse=dt*S;
h.normalizationResidual=sum(f)-1;
end
