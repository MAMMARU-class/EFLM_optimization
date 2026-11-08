function h = horizontal_grf_shape(a,cfg)
% Time-independent Hermite shape; beta = dt * df/dt.
n=cfg.grfN; k=cfg.grfK; j=cfg.grfJ;
nodes=setdiff(2:n-1,[k j]);
f=zeros(1,n); f(nodes)=a(7:n+2); f(j)=1; f(1:k-1)=-f(1:k-1);
beta=zeros(1,n); beta([1 n])=a(end);
beta(2:n-1)=(f(3:n)-f(1:n-2))/2; beta(j)=0;
y0=f(1:end-1); y1=f(2:end); b0=beta(1:end-1); b1=beta(2:end);
c=[(2*y0-2*y1+b0+b1)',(-3*y0+3*y1-2*b0-b1)',b0',y0'];
S=sum(f(2:end-1))+(beta(1)-beta(end))/12;
CH=sum((1:n-2).*f(2:end-1))+beta(1)/30+sum(beta(2:end-1))/15-(5*n-7)*beta(end)/60;
h=struct('values',f,'beta',beta,'coefficients',c,'S',S,'CH',CH,'ratioNodes',nodes);
end
