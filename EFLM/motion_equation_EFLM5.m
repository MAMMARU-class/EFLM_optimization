function equations = motion_equation_EFLM5()
% Optional symbolic derivation, matching urls/motion_equation.m procedure.
% Not executed during optimization. Requires Symbolic Math Toolbox.
syms x(t) z(t) h(t) k(t) a(t)
syms m g L1 L2 L3 L4 LB LD ks Fx Fz th tk preload real
q=[x(t);z(t);h(t);k(t);a(t)];
ell=sqrt(L2^2+LB^2+LD^2+2*L2*(LD*cos(k)-LB*cos(a))-2*LB*LD*cos(k-a));
s=ell-L3;
T=m*(diff(x,t)^2+diff(z,t)^2)/2;
U=m*g*z+ks*s^2/2; L=T-U;
toe=[x+L1*cos(h)+L2*cos(h+k)+(L4-LB)*cos(h+k-a);...
 z-L1*sin(h)-L2*sin(h+k)-(L4-LB)*sin(h+k-a)];
JT=[functionalDerivative(toe(1),q),functionalDerivative(toe(2),q)];
extra=[0;0;0;preload*LB/ell*(L2*sin(k)-LD*sin(k-a));...
 -preload*LD/ell*(L2*sin(k)-LB*sin(k-a))];
lhs=-functionalDerivative(L,q);
rhs=[0;0;th;tk;0]+JT*[Fx;Fz]+extra;
equations=struct('q',q,'T',T,'U',U,'L',L,'toe',toe,...
 'contactJacobianTranspose',JT,'extraTorque',extra,'lhs',lhs,'rhs',rhs,...
 'stance',lhs==rhs,'contact',toe==0);
% Substitute preload = p.preload*smoothstep(s/p.rise) after differentiation:
% this is urls' nonconservative additional generalized force, not part of U.
end
