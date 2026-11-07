function [result, folder] = run_stage1_optimization(overrides)
clc;
% Usage: run_stage1_optimization() or run_stage1_optimization(options).
useDefaults=nargin<1;
root=fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root,'common')));
if useDefaults
    % Each decision variable is [lower, upper, initial].
    overrides = struct;
    overrides.AmpH = [5, 35, 10];                  % [N]
    overrides.thetaL = deg2rad([30, 60, 50]);      % [rad]
    overrides.v = [0.4, 4.0, 1.0];                % [m/s]
    overrides.Duty = [0.1, 0.60, 0.10];          % [-]
    overrides.r0 = [0.25, 0.33, 0.30];            % [m]
    overrides.stanceScale = [0.5, 2.0, 1.0];       % [-]

    % 
    overrides.starts = 1000;
    overrides.localStarts = 200;
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
