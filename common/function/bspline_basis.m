function B=bspline_basis(u,knots,degree)
% Cox-de Boor basis; clamped right endpoint evaluated by its left limit.
u=u(:); right=u==knots(end); u(right)=knots(end)-eps(knots(end));
B=double(u>=knots(1:end-1) & u<knots(2:end));
for p=1:degree
 C=zeros(numel(u),numel(knots)-p-1);
 for i=1:size(C,2)
  a=knots(i+p)-knots(i); b=knots(i+p+1)-knots(i+1);
  if a>0, C(:,i)=C(:,i)+(u-knots(i))/a.*B(:,i); end
  if b>0, C(:,i)=C(:,i)+(knots(i+p+1)-u)/b.*B(:,i+1); end
 end
 B=C;
end
if degree==3, B(right,:)=0; B(right,end)=1; end
end
