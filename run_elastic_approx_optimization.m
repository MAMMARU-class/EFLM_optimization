function [result, folder] = run_elastic_approx_optimization(overrides)
%RUN_ELASTIC_APPROX_OPTIMIZATION Project-root entry point.
%   [RESULT,FOLDER] = RUN_ELASTIC_APPROX_OPTIMIZATION() uses defaults.
%   Supply a struct to override any field in optimization_settings.m.
%
if nargin < 1
    % Settings used when this function is run without an input argument.
    % Variable order: AmpH [N], thetaL [rad], v [m/s], Duty [-].
    overrides = struct;
    overrides.lower = [0, deg2rad(1), 0.10, 0.10];
    overrides.upper = [80, deg2rad(50), 2.50, 0.80];
    overrides.initial = [0.5, deg2rad(8), 0.473, 0.50];
    overrides.starts = 80;
    overrides.localStarts = 20;
end
validateattributes(overrides, {'struct'}, {'scalar'}, mfilename, 'overrides');

% The MATLAB project exposes every project file. No path mutation is needed here.
[result, folder] = main_elastic_approx(overrides);
end
