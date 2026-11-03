function [result, folder] = run_elastic_optimization(model,overrides)
clc;
% Usage: run_elastic_optimization('approx') or ('exact',options).
if nargin<1, model='approx'; end
useDefaults=nargin<2;
if isstruct(model), overrides=model; model='approx'; useDefaults=false; end
root=fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root,'common')));
if useDefaults
    % Shared presets used when no overrides struct is supplied.
    % Variable order: AmpH [N], thetaL [rad], v [m/s], Duty [-], r0 [m].
    p = robot_parameters();
    overrides = struct;
    overrides.lower = [1, deg2rad(1), 0.4, 0.01, 0.22];
    overrides.upper = [30, deg2rad(80), 7.0, 0.60, 0.32];
    overrides.initial = [4, deg2rad(50), 1.0, 0.10, p.leg];
    overrides.starts = 1000;
    overrides.localStarts = 100;
    % Objective: maximize w1*meanSpeed - w2*CoT.
    overrides.w1 = 2;
    overrides.w2 = 1;
end
validateattributes(overrides, {'struct'}, {'scalar'}, mfilename, 'overrides');

% Both models use the same settings and optimization pipeline.
[result, folder] = main_elastic(model,overrides);
end
