function [result, folder] = run_stage1_optimization(overrides)
clc;
% Usage: run_stage1_optimization() or run_stage1_optimization(options).
% Each continuous parameter is [lower, upper, initial].
root=fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root,'common')));
if nargin<1
    overrides=struct;
    % Basic initial state.
    overrides.vx0 = [1.8, 4.5, 3.0];              % [m/s]
    overrides.thetaL = deg2rad([40, 70, 50]);      % [rad]
    overrides.r0 = [0.27, 0.33, 0.30];            % [m]
    % Timing.
    overrides.stanceScale = [0.5, 2.0, 1.0];       % alpha [-]
    overrides.duty = [0.2, 0.60, 0.50];           % D [-]
    % Horizontal GRF. Previous trajectories: Ix approximately 1.13--1.99 N s.
    overrides.grfXN = 12;                         % control points (>=5)
    overrides.Ix = [0.6, 3.0, 1.13];              % [N s]
    overrides.grfXBetaStart = [-0.5, 0, 0];        % independent beta_1 [-]
    overrides.grfXBetaEnd = [-0.5, 0, 0];          % independent beta_n [-]
    overrides.restrictGrfK = true;               % true: use grfKSelection
    overrides.grfKSelection = [7, 8];       % otherwise all 3:n-2
    % Vertical GRF.
    overrides.grfZN = 9;
    overrides.grfZBetaStart = [0, 0.5, 0];
    overrides.grfZBetaEnd = [-0.5, 0, 0];
    % Constraints
    overrides.minGaitDuration = 0.25; % minimum T(=T_st + T_fl) [s]
    % Search effort: total = startsPerK * number of selected k values.
    overrides.startsPerK = 2500000;
    overrides.localStarts = 1000;
    overrides.useParallel = true;
    overrides.parallelWorkers = 20;
    % Weighted squared relative errors from the targets.
    overrides.targetSpeed = 5;                    % cycle-mean speed [m/s]
    overrides.targetCoT = 0.3;
    overrides.w1 = 5;
    overrides.w2 = 1;
    overrides.resumeFolder = '';                 % existing run folder to resume
end
validateattributes(overrides,{'struct'},{'scalar'},mfilename,'overrides');
[result,folder]=main_stage1(overrides);
end
