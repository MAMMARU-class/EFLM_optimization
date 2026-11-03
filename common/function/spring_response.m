function [force,stiffness,energy] = spring_response(s,p)
% Smooth preload with linear stiffness present everywhere. Negative s is
% a solver continuation only; it is never an admissible physical state.
u=max(0,min(1,s/p.rise));
rho=u.^2.*(3-2*u);
force=p.ks*s+p.preload*rho;
stiffness=p.ks+6*p.preload/p.rise*u.*(1-u);
energy=.5*p.ks*s.^2+p.preload*p.rise*(u.^3-.5*u.^4)...
    +p.preload*max(s-p.rise,0);
end
