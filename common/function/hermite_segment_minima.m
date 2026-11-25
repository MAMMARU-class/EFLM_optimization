function minima=hermite_segment_minima(op,controls)
% Exact minima of the cubic on each interval, including stationary points.
n=size(op.coefficients,3); minima=zeros(n,1);
for j=1:n
 c=op.coefficients(:,:,j)*controls;
 q=[0;1];
 if abs(c(4))>1e-14
  disc=4*c(3)^2-12*c(4)*c(2);
  if disc>=0, q=[q;(-2*c(3)+sqrt(disc))/(6*c(4));(-2*c(3)-sqrt(disc))/(6*c(4))]; end
 elseif abs(c(3))>1e-14
  q=[q;-c(2)/(2*c(3))];
 end
 q=q(q>=0 & q<=1);
 minima(j)=min(c(1)+q.*(c(2)+q.*(c(3)+q*c(4))));
end
end
