function [force,stiffness,energy] = spring_response(s,p)
% Preloaded spring measured from its preloaded length (s=0).
% Negative s is a smooth Newton continuation; admissible states satisfy s>=0.
force=p.preload+p.ks*s;
stiffness=p.ks+zeros(size(s));
energy=p.preload*s+.5*p.ks*s.^2;
end
