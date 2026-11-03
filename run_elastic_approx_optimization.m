function [result, folder] = run_elastic_approx_optimization(overrides)
clc;
%RUN_ELASTIC_APPROX_OPTIMIZATION Project-root entry point.
%   [RESULT,FOLDER] = RUN_ELASTIC_APPROX_OPTIMIZATION() uses defaults.
%   Supply a struct to override any field in optimization_settings.m.
%
if nargin < 1
    % Settings used when this function is run without an input argument.
    % Variable order: AmpH [N], thetaL [rad], v [m/s], Duty [-].
    overrides = struct;
    overrides.lower = [1, deg2rad(1), 0.4, 0.01];
    overrides.upper = [30, deg2rad(80), 7.0, 0.60];
    overrides.initial = [4, deg2rad(50), 1.0, 0.10];
    overrides.starts = 500;
    overrides.localStarts = 80;
    % Objective: maximize w1*meanSpeed - w2*CoT.
    overrides.w1 = 1;
    overrides.w2 = 50;
end
validateattributes(overrides, {'struct'}, {'scalar'}, mfilename, 'overrides');

% The MATLAB project exposes every project file. No path mutation is needed here.
[result, folder] = main_elastic_approx(overrides);
end
