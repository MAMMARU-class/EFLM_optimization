function [e,d] = eflm5_balance(q,Y,GRF,p)
% Euler-Lagrange: M qdd + grad U = tau + Jtoe' GRF + tau_ls.
% Keep urls' additional torque formula literally (not replaced by -Fs*ds/dq).
d=eflm5_geometry(q,p); s=d.s; k=q(:,2); a=q(:,3);
u=s/p.rise; u(real(u)<=0)=0; u(real(u)>=1)=1;
preload=p.preload*u.^2.*(3-2*u);
% Linear spring remains present at every extension, as in the shared model.
d.gradU=p.ks*s.*d.ds;
d.extraForce=preload;
d.extraTorque=[zeros(size(s)),...
 preload.*p.LB./d.length.*(p.L2*sin(k)-p.LD*sin(k-a)),...
 -preload.*p.LD./d.length.*(p.L2*sin(k)-p.LB*sin(k-a))];
d.contact=d.Jx.*GRF(:,1)+d.Jz.*GRF(:,2);
d.requiredTorque=d.gradU-d.contact-d.extraTorque;
e=[d.toe-Y,d.requiredTorque(:,3)];
end
