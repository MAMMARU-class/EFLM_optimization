function h = horizontal_impulse_shape(a,cfg)
% Independent endpoint beta, normalized knot magnitudes, no maximum knot j.
n=cfg.grfXN; k=cfg.grfK; ix=cfg.index;
nodes=setdiff(2:n-1,k);
f=zeros(1,n); f(nodes)=a(ix.fx); f(1:k-1)=-f(1:k-1);
beta=zeros(1,n); beta(1)=a(ix.betaXStart); beta(end)=a(ix.betaXEnd);
beta(2:n-1)=(f(3:n)-f(1:n-2))/2;
y0=f(1:end-1); y1=f(2:end); b0=beta(1:end-1); b1=beta(2:end);
c=[(2*y0-2*y1+b0+b1)',(-3*y0+3*y1-2*b0-b1)',b0',y0'];
S0=sum(f)+(beta(1)-beta(end))/12;
Sx=sum(abs(f))+(2*beta(k)-beta(1)-beta(end))/12;
CH=sum((1:n-2).*f(2:end-1))+beta(1)/30+sum(beta(2:end-1))/15-(5*n-7)*beta(end)/60;
h=struct('values',f,'beta',beta,'coefficients',c,'S0',S0,'Sx',Sx,...
 'CH',CH,'valueNodes',nodes,'normalizationResidual',sum(abs(f))-1);
end
