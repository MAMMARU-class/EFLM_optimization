function [result, folder] = run_stage1_optimization(overrides)
clc;
% Usage: run_stage1_optimization(), run_stage1_optimization(method),
% or run_stage1_optimization(options) with options.method set.
useDefaults=nargin<1 || ischar(overrides) || (isstring(overrides) && isscalar(overrides));
if nargin>0 && useDefaults, selectedMethod=char(overrides); end
root=fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root,'common')));
if useDefaults
    % Shared presets used when no overrides struct is supplied.
    % Variable order: AmpH [N], thetaL [rad], v [m/s], Duty [-], r0 [m].
    p = robot_parameters();
    overrides = struct;
    overrides.method = 'mean_velocity'; % 'mean_velocity' or 'initial_velocity'
    overrides.lower = [1, deg2rad(1), 0.4, 0.01, 0.22];
    overrides.upper = [30, deg2rad(80), 7.0, 0.60, 0.34];
    overrides.initial = [4, deg2rad(50), 1.0, 0.10, p.leg];
    overrides.starts = 500;
    overrides.localStarts = 100;
    overrides.useParallel = true;
    overrides.parallelWorkers = 16;
    % Objective: maximize w1*meanSpeed - w2*CoT.
    overrides.w1 = 2;
    overrides.w2 = 1;
end
if nargin>0 && useDefaults, overrides.method=selectedMethod; end
validateattributes(overrides, {'struct'}, {'scalar'}, mfilename, 'overrides');

[result, folder] = main_stage1(overrides);
end
