function [result, folder] = run_stage1_optimization(overrides)
clc;
% Usage: run_stage1_optimization() or run_stage1_optimization(options).
useDefaults=nargin<1;
root=fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root,'common')));
if useDefaults
    % Each decision variable is [lower, upper, initial].
    overrides = struct;
    overrides.AmpH = [1, 35, 10];                  % [N]
    overrides.grfN = 19;                          % number of Hermite control points (>=4)
    overrides.grfSlope = [-40, 40, 0];            % [lower, upper, fallback] [s^-1]; interior seeds use central differences
    overrides.thetaL = deg2rad([30, 65, 50]);      % [rad]
    overrides.v = [0.4, 4.0, 1.0];                % [m/s]
    overrides.Duty = [0.1, 0.60, 0.10];          % [-]
    overrides.r0 = [0.25, 0.33, 0.30];            % [m]
    overrides.stanceScale = [0.5, 2.0, 1.0];       % [-]

    % 
    overrides.starts = 1500 * 16*16;
    overrides.localStarts = 500;
    overrides.useParallel = true;
    overrides.parallelWorkers = 16;

    % 
    % Minimize weighted squared relative errors from these targets.
    overrides.targetSpeed = 4; % [m/s], cycle-mean horizontal speed
    overrides.targetCoT = 0.3; % dimensionless
    % w1: speed error, w2: CoT error
    overrides.w1 = 5;
    overrides.w2 = 1;
end
validateattributes(overrides, {'struct'}, {'scalar'}, mfilename, 'overrides');

[result, folder] = main_stage1(overrides);
end
