function [force,stiffness,energy] = spring_response(s,p,mode)
% Preloaded spring measured from its preloaded length (s=0).
% Negative s is a smooth Newton continuation; admissible states satisfy s>=0.
% The five-coordinate EFLM retains its previous diagnostic spring channels.
if nargin<3, mode='preloaded'; end
if strcmp(mode,'smoothDiagnostic')
 u=max(0,min(1,s/p.rise));
 rho=u.^2.*(3-2*u);
 force=p.ks*s+p.preload*rho;
 stiffness=p.ks+6*p.preload/p.rise*u.*(1-u);
 energy=.5*p.ks*s.^2+p.preload*p.rise*(u.^3-.5*u.^4)...
     +p.preload*max(s-p.rise,0);
else
 force=p.preload+p.ks*s;
 stiffness=p.ks+zeros(size(s));
 energy=p.preload*s+.5*p.ks*s.^2;
end
end
