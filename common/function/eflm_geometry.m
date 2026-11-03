function [Y,qa,points] = eflm_geometry(h,k,s,p)
% Shared exact EFLM assembly branch. Vectorized, complex-step-compatible geometry.
d=sqrt(p.L2^2+p.LD^2+2*p.L2*p.LD*cos(k));
qa=acos((p.L2^2+d.^2-p.LD^2)./(2*p.L2*d))...
  +acos((p.LB^2+d.^2-(p.L3+s).^2)./(2*p.LB*d));
B=[p.L1*cos(h),-p.L1*sin(h)];
A=[(p.L1-p.LD)*cos(h),-(p.L1-p.LD)*sin(h)];
C=B+[p.L2*cos(h+k),-p.L2*sin(h+k)];
e=[cos(h+k-qa),-sin(h+k-qa)];
Y=C+(p.L4-p.LB)*e;
if nargout>2
 points=cat(3,zeros(size(A)),A,B,C,C-p.LB*e,Y);
end
end
