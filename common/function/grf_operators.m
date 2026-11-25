function op=grf_operators(kind,count,u)
% Shared linear basis: force, d/du, integral, and double integral.
if strcmp(kind,'bspline'), op=bspline_operators(count,u); return; end
persistent keys values
if isempty(keys), keys={}; values={}; end
key={count,u(:)};
for j=1:numel(keys), if isequal(keys{j},key), op=values{j}; return; end, end
h=1/(count-1); E=eye(count); slopes=zeros(count);
slopes(1,:)=E(2,:)-E(1,:); slopes(end,:)=E(end,:)-E(end-1,:);
slopes(2:end-1,:)=(E(3:end,:)-E(1:end-2,:))/2;
% Segment coefficients in local coordinate q=(u-u_j)/h, ascending powers.
P=zeros(4,count,count-1);
for j=1:count-1
 P(:,:,j)=[E(j,:);slopes(j,:);...
  -3*E(j,:)+3*E(j+1,:)-2*slopes(j,:)-slopes(j+1,:);...
  2*E(j,:)-2*E(j+1,:)+slopes(j,:)+slopes(j+1,:)];
end
u=u(:); op.B=zeros(numel(u),count); op.D=op.B; op.I=op.B; op.J=op.B;
for j=1:count-1
 q=min(1,max(0,(u-(j-1)*h)/h));
 area=h*[q,q.^2/2,q.^3/3,q.^4/4]*P(:,:,j);
 moment=h^2*[q.^2/2,q.^3/3,q.^4/4,q.^5/5]*P(:,:,j);
 op.I=op.I+area;
 op.J=op.J+(u-(j-1)*h).*area-moment;
 mask=u>=(j-1)*h & (u<j*h | j==count-1);
 if any(mask)
  v=q(mask); op.B(mask,:)=[ones(size(v)),v,v.^2,v.^3]*P(:,:,j);
  op.D(mask,:)=[zeros(size(v)),ones(size(v)),2*v,3*v.^2]*P(:,:,j)/h;
 end
end
op.coefficients=P; op.knots=linspace(0,1,count);
if numel(keys)>=12, keys=keys(2:end); values=values(2:end); end
keys{end+1}=key; values{end+1}=op;
end
