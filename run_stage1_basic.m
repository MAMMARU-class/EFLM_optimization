function [summary,folder] = run_stage1_basic(overrides)
clc;
% Basic grid parameters: [center halfWidth oddCount]; other parameters: [lower upper initial].
root=fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root,'common')),fullfile(root,'stage1_basic'));
if nargin<1, overrides=struct; end
    options=struct;
    % basic options for grid search
    options.vx0=[2.4 0.5 7];                         % [m/s], center +/- width, odd count
    options.thetaL=[deg2rad(45) deg2rad(7) 7];      % [rad], convert center/width only
    options.r0=[0.28 0.04 7];                         % [m], center +/- width, odd count
    % time option
    options.stanceScale=[0.6 1.5 1];                    % continuous [lower upper initial]
    options.duty=[0.35 0.65 0.5];
    % GRF x
    options.grfXN=10;
    options.Ix=[0.3 3 1.13];                          % [N s]
    options.grfXBetaStart=[-0.3 0 0];
    options.grfXBetaEnd=[-0.3 0 0];
    options.restrictGrfK=true;                       % pre_defined range
    options.grfKSelection=6;             % used only if restrictGrfK=true
    % GRF z
    options.grfZN=7;
    options.grfZBetaStart=[0 0.5 0];
    options.grfZBetaEnd=[-0.5 0 0];

    % search and save settings
    options.startsPerK=300;                        % per BASIC GRID POINT and k
    options.nodes=121;                              % broad evaluation only
    options.topResults=3;                           % global feasible top N: PNG and MP4
    options.useParallel=true;
    options.parallelWorkers=20;
    % cost function
    % J_cost = w1* ( \bar{v} / target_v ) + w2* ( CoT / target_CoT )
    options.targetSpeed=4;
    options.targetCoT=.3;
    options.w1=5;
    options.w2=1;

    validateattributes(overrides,{'struct'},{'scalar'});

for name=fieldnames(overrides)', options.(name{1})=overrides.(name{1}); end
[summary,folder]=main_stage1_basic(options);
end
