function [result, folder] = run_stage1_optimization(overrides)
clc;
% Usage: run_stage1_optimization() or run_stage1_optimization(options).
useDefaults=nargin<1;
root=fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root,'common')));
if useDefaults
    % Each decision variable is [lower, upper, initial].
    overrides = struct;
    overrides.AmpH = [10, 35, 20];                  % [N]
    overrides.grfN = 15;                          % number of Hermite control points (>=5)
    overrides.grfZN = 9;                          % vertical control points (>=3)
    overrides.grfZBetaStart = [0, 0.5, 0];         % dimensionless [lower, upper, initial]
    overrides.grfZBetaEnd = [-0.5, 0, 0];          % dimensionless [lower, upper, initial]

    % 
    overrides.restrictGrfPairs = false;            % search only the selected [k,j] rows
    overrides.grfPairSelection = [ ...
        7, 4; 7, 5; 7, 6; 7, 8; 7, 9; 7, 10; ...
        8, 5; 8, 6; 8, 7; 8, 9; 8, 10; 8, 11; ...
        9, 6; 9, 7; 9, 8; 9, 10; 9, 11; 9, 12; ...
        10, 7; 10, 8; 10, 9; 10, 11; 10, 12; 10, 13; ...
    ];

    % 
    overrides.grfBetaEndpoint = [-0.4, 0, 0];    % shared dimensionless beta [lower, upper, initial]
    overrides.thetaL = deg2rad([40, 65, 50]);      % [rad]
    overrides.v = [1.0, 3.5, 1.0];                % [m/s]
    overrides.Duty = [0.3, 0.60, 0.50];          % [-]
    overrides.r0 = [0.26, 0.33, 0.30];            % [m]
    overrides.stanceScale = [0.5, 2.0, 1.0];       % [-]

    % 
    startsPerPair = 2000;
    if overrides.restrictGrfPairs
        pairCount = size(unique(overrides.grfPairSelection,'rows'),1);
    else
        pairCount = (overrides.grfN-4)*(overrides.grfN-3);
    end
    overrides.starts = startsPerPair * pairCount;
    overrides.localStarts = 1000;
    overrides.useParallel = true;
    overrides.parallelWorkers = 20;

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
