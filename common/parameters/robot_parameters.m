function p = robot_parameters()
% Numeric values from sim_env_EFLM_old/parameters (SI, joint-side motors).
p.g=9.81; 
p.m=(4034.447/4+751.233+129.422+12+36.147)/1000;

p.L1=.130; 
p.L2=.140; 
p.L3=.135; 
p.L4=.110; 
p.LB=.030; 
p.LD=.023;

p.ks=17430; 
p.preload=3.4; 
p.smax=.015; 
p.damping=0;

% p.jointLower=deg2rad([55 16]);
% p.jointUpper=deg2rad([155 144]);
p.jointLower = deg2rad([10 16]);
p.jointUpper = deg2rad([170 144]);

p.tauMax=9; 
p.rpmConstant=155; 
p.rpmZero=285;
p.rpmLimit=415;
p.tnSlope=.0692308; 

p.leg=.290;
p.lengthScale=(.25+.34)/2; % Fixed numerical normalization, not optimized r0.
end
