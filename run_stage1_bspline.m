function [result,folder]=run_stage1_bspline(overrides)
% bspline: independent stance time, shared x/z controls and zero net Fx impulse.
% Resume with struct('resumeFolder',folder); optionally 'finishBroadEarly',true.
if nargin<1
    overrides=struct;
    % Each basic parameter is [lower, upper, initial].
    overrides.vx0 = [1.0, 3.5, 1.0];          % horizontal speed [m/s]
    overrides.thetaL = deg2rad([40,65,50]);   % initial leg angle [rad]
    overrides.r0 = [0.26,0.33,0.30];          % initial leg length [m]
    overrides.stanceTime = [0.05,0.25,0.15];  % independent stance time [s]
    overrides.duty = [0.3,0.6,0.5];           % D = T_st / T
    overrides.minGaitDuration = 0.25;        % minimum T_st + T_fl [s]

    % Both axes use the same basis and zero endpoint forces.
    % Hermite endpoint slopes are one-sided differences; B-spline is cubic.
    overrides.controlCount = 11;
    % mu=0.4 in robot_parameters; Fx limits follow the vertical controls.

    overrides.broadStarts = 300000;
    overrides.localStarts = 10000;
    overrides.save_interval = 2000;          % completed starts per checkpoint
    overrides.resumeFolder = '';
    overrides.nodes = 121;                   % broad stance evaluation nodes
    overrides.narrowNodes = 1201;
    overrides.finalNodes = 1201;
    overrides.useParallel = true;
    overrides.parallelWorkers = 20;
    overrides.localRadius = 0.15;            % normalized narrow-box radius
    overrides.seed = 20261103;

    overrides.targetSpeed = 4;               % cycle-average speed [m/s]
    overrides.targetCoT = 0.3;
    overrides.w1 = 5;
    overrides.w2 = 1;
    overrides.video = true;
end
root=fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root,'common')),fullfile(root,'stage1_point_mass'));
[result,folder]=run_grf_optimization('bspline',overrides,root);
end
