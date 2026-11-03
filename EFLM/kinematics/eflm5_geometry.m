function d = eflm5_geometry(q,p)
% Independent hip/knee/ankle coordinates, as in urls/parameter.m. phi=0.
h=q(:,1); k=q(:,2); a=q(:,3); H=h+k; A=H-a; c=p.L4-p.LB;
d.toe=[p.L1*cos(h)+p.L2*cos(H)+c*cos(A),...
 -p.L1*sin(h)-p.L2*sin(H)-c*sin(A)];
d.Jx=[-p.L1*sin(h)-p.L2*sin(H)-c*sin(A),-p.L2*sin(H)-c*sin(A),c*sin(A)];
d.Jz=[-p.L1*cos(h)-p.L2*cos(H)-c*cos(A),-p.L2*cos(H)-c*cos(A),c*cos(A)];
d.length=sqrt(p.L2^2+p.LB^2+p.LD^2+2*p.L2*(p.LD*cos(k)-p.LB*cos(a))-2*p.LB*p.LD*cos(k-a));
d.s=d.length-p.L3;
d.ds=[zeros(size(k)),(-p.L2*p.LD*sin(k)+p.LB*p.LD*sin(k-a))./d.length,...
 (p.L2*p.LB*sin(a)-p.LB*p.LD*sin(k-a))./d.length];
B=[p.L1*cos(h),-p.L1*sin(h)]; C=B+[p.L2*cos(H),-p.L2*sin(H)];
anchor=[(p.L1-p.LD)*cos(h),-(p.L1-p.LD)*sin(h)];
d.points=cat(3,zeros(size(B)),anchor,B,C,C-p.LB*[cos(A),-sin(A)],d.toe);
end
