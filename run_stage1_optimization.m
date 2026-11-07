function [result, folder] = run_stage1_optimization(overrides)
clc;
% Usage: run_stage1_optimization() or run_stage1_optimization(options).
useDefaults=nargin<1;
root=fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root,'common')));
if useDefaults
    % Shared presets used when no overrides struct is supplied.
    % Order: AmpH [N], thetaL [rad], v [m/s], Duty [-], r0 [m], stanceScale [-].
    overrides = struct;

    % 
    
    % 
    overrides.lower = [1, deg2rad(1), 0.4, 0.01, 0.22, 0.5];
    overrides.upper = [30, deg2rad(80), 7.0, 0.60, 0.34, 2.0];
    overrides.initial = [4, deg2rad(50), 1.0, 0.10, 0.30, 1.0];

    % 
    overrides.starts = 500;
    overrides.localStarts = 100;
    overrides.useParallel = true;
    overrides.parallelWorkers = 16;

    % 
    % Minimize weighted squared relative errors from these targets.
    overrides.targetSpeed = 4; % [m/s], cycle-mean horizontal speed
    overrides.targetCoT = 0.3; % dimensionless
    % w1: speed error, w2: CoT error
    overrides.w1 = 2;
    overrides.w2 = 1;
end
validateattributes(overrides, {'struct'}, {'scalar'}, mfilename, 'overrides');

[result, folder] = main_stage1(overrides);
end
