function op=bspline_operators(count,u)
% Cubic open-uniform basis and exact polynomial quadrature. Cache fixed grids.
persistent keys values
if isempty(keys), keys={}; values={}; end
key={count,u(:)};
for j=1:numel(keys), if isequal(keys{j},key), op=values{j}; return; end, end
knots=[zeros(1,4),(1:count-4)/(count-3),ones(1,4)];
u=u(:); op.B=bspline_basis(u,knots,3);
L=bspline_basis(u,knots,2); op.D=zeros(numel(u),count);
for i=1:count
 a=knots(i+3)-knots(i); b=knots(i+4)-knots(i+1);
 if a>0, op.D(:,i)=op.D(:,i)+3/a*L(:,i); end
 if b>0, op.D(:,i)=op.D(:,i)-3/b*L(:,i+1); end
end
op.I=zeros(numel(u),count); op.J=op.I;
% Four-point Gauss integrates u*B (degree four) exactly on each knot span.
g=[-.8611363115940526;-.3399810435848563;.3399810435848563;.8611363115940526];
w=[.3478548451374539;.6521451548625461;.6521451548625461;.3478548451374539];
breaks=unique(knots);
for j=1:numel(u)
 for s=1:numel(breaks)-1
  lo=breaks(s); hi=min(u(j),breaks(s+1)); if hi<=lo, continue; end
  q=(lo+hi)/2+(hi-lo)/2*g; W=(hi-lo)/2*w;
  B=bspline_basis(q,knots,3);
  op.I(j,:)=op.I(j,:)+W'*B;
  op.J(j,:)=op.J(j,:)+(W.*(u(j)-q))'*B;
 end
end
op.knots=knots;
if numel(keys)>=12, keys=keys(2:end); values=values(2:end); end
keys{end+1}=key; values{end+1}=op;
end
