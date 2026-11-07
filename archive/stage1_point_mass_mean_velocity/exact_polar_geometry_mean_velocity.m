function d = exact_polar_geometry_mean_velocity(k,s,p)
% Exact EFLM polar geometry at hip angle zero. First derivatives are analytic.
% Complex inputs are supported for differentiating Rs and Bs (Newton Hessian).
ck=cos(k); sk=sin(k); S=p.L3+s; L=p.L4-p.LB;
D=sqrt(p.L2^2+p.LD^2+2*p.L2*p.LD*ck);
Dk=-p.L2*p.LD*sk./D;
c=(p.LB^2+D.^2-S.^2)./(2*p.LB*D);
if isreal(k) && isreal(s)
 c(abs(c)>=1 | S<=0)=NaN; % Invalid closure, never clip the mechanism.
end
root=sqrt(1-c.^2);
qa=atan(p.LD*sk./(p.L2+p.LD*ck))+acos(c);
qak=p.LD*(p.L2*ck+p.LD)./D.^2 ...
 -(1-(p.LB^2-S.^2)./D.^2).*Dk./(2*p.LB*root);
qas=S./(p.LB*D.*root);
u=k-qa; cu=cos(u); su=sin(u);
x=p.L1+p.L2*ck+L*cu; z=-p.L2*sk-L*su;
xk=-p.L2*sk-L*su.*(1-qak); zk=-p.L2*ck-L*cu.*(1-qak);
xs=L*su.*qas; zs=L*cu.*qas;
R=sqrt(x.^2+z.^2);
d=struct('R',R,'beta',atan2(-real(z),real(x)),...
 'Rk',(x.*xk+z.*zk)./R,'Rs',(x.*xs+z.*zs)./R,...
 'Bk',(z.*xk-x.*zk)./R.^2,'Bs',(z.*xs-x.*zs)./R.^2,...
 'qa',qa,'qak',qak,'qas',qas);
end
