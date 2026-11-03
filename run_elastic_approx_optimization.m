function [result, folder] = run_elastic_approx_optimization(overrides)
%RUN_ELASTIC_APPROX_OPTIMIZATION Project-root entry point.
%   [RESULT,FOLDER] = RUN_ELASTIC_APPROX_OPTIMIZATION() uses defaults.
%   Supply a struct to override any field in optimization_settings.m.
%
%   Example: change broad bounds and MultiStart counts.
%     options.lower = [0, deg2rad(1), 0.10, 0.20];
%     options.upper = [30, deg2rad(25), 1.50, 0.80];
%     options.initial = [0.5, deg2rad(8), 0.473, 0.50];
%     options.starts = 24;
%     options.localStarts = 8;
%     [result, folder] = run_elastic_approx_optimization(options);

if nargin < 1
    overrides = struct;
end
validateattributes(overrides, {'struct'}, {'scalar'}, mfilename, 'overrides');

% The MATLAB project exposes every project file. No path mutation is needed here.
[result, folder] = main_elastic_approx(overrides);
end
