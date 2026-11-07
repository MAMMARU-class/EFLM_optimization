function [a,check] = initialize_horizontal_grf_slopes(a,cfg,lower,upper)
% Initialize a batch of starts using forward secants in physical time.
% Only slope entries are changed; all other variables and search bounds stay intact.
if nargin<3, lower=cfg.lower; upper=cfg.upper; end
n=cfg.grfN; k=cfg.grfK; j=cfg.grfJ; count=size(a,1);
ratioNodes=setdiff(2:n-1,[k j]); slopeNodes=setdiff(1:n,j);
slopeColumns=(7+n-4):size(a,2);
values=zeros(count,n); values(:,ratioNodes)=a(:,7:6+n-4);
values(:,j)=1; values(:,1:k-1)=-values(:,1:k-1);
dt=(a(:,6).*2.*a(:,5).*sin(a(:,2))./a(:,3))/(n-1);
valid=isfinite(dt) & dt>0;
safeDt=dt; safeDt(~valid)=1;
secant=diff(values,1,2)./safeDt;
slopes=[secant,secant(:,end)]; % Last point uses the preceding interval.
slopes(:,slopeNodes)=min(upper(slopeColumns),max(lower(slopeColumns),slopes(:,slopeNodes)));
slopes(:,j)=0;
slopes(~valid,slopeNodes)=a(~valid,slopeColumns);
a(:,slopeColumns)=slopes(:,slopeNodes);

% Exact shape checks over each cubic, vectorized across starts and intervals.
y0=values(:,1:end-1); y1=values(:,2:end);
b0=slopes(:,1:end-1); b1=slopes(:,2:end);
c3=2*y0-2*y1+safeDt.*(b0+b1);
c2=-3*y0+3*y1-safeDt.*(2*b0+b1);
c1=safeDt.*b0; c0=y0;
direction=ones(1,n-1); direction(1:k-1)=-1;
signViolation=max(-direction.*y0,-direction.*y1);
amplitudeViolation=max(abs(y0),abs(y1))-1;
quadratic=abs(c3)>1e-14;
linear=~quadratic & abs(c2)>1e-14;
discriminant=4*c2.^2-12*c3.*c1;
denominator=6*c3; denominator(~quadratic)=1;
radical=sqrt(max(0,discriminant));
root1=(-2*c2-radical)./denominator;
root2=(-2*c2+radical)./denominator;
root1(~(quadratic & discriminant>=0))=NaN;
root2(~(quadratic & discriminant>=0))=NaN;
root1(linear)=-c1(linear)./(2*c2(linear));
for index=1:2
 if index==1, u=root1; else, u=root2; end
 interior=isfinite(u) & u>0 & u<1;
 u(~interior)=0;
 value=((c3.*u+c2).*u+c1).*u+c0;
 violation=1e-4-direction.*value; violation(~interior)=-Inf;
 signViolation=max(signViolation,violation);
 violation=abs(value)-1; violation(~interior)=-Inf;
 amplitudeViolation=max(amplitudeViolation,violation);
end
meanArea=c3/4+c2/3+c1/2+c0;
check.shapeViolation=max([zeros(count,1),signViolation,amplitudeViolation,...
 1e-4-direction.*meanArea],[],2);
check.normalizedImpulse=(sum(values(:,2:end-1),2)+safeDt.*(slopes(:,1)-slopes(:,end))/12)/(n-1);
check.shapeViolation(~valid)=Inf; check.normalizedImpulse(~valid)=Inf;
check.feasible=valid & check.shapeViolation<=cfg.constraintTolerance & ...
 abs(check.normalizedImpulse)<=cfg.constraintTolerance;
end
