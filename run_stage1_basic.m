function [summary,folder] = run_stage1_basic(overrides)
clc;
% Basic grid parameters: [center halfWidth oddCount]; other parameters: [lower upper initial].
root=fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root,'common')),fullfile(root,'stage1_basic'));
if nargin<1, overrides=struct; end
    options=struct;
    % basic options for grid search
    options.vx0=[2.05 0.45 7];                       % [m/s], 1.6 to 2.5
    options.thetaL=[deg2rad(42.5) deg2rad(7.5) 7];  % [rad], 35 to 50 deg
    options.r0=[0.28 0.04 7];                        % [m], 0.24 to 0.32
    % time option
    options.stanceScale=[0.6 1.5 1];                    % continuous [lower upper initial]
    options.duty=[0.35 0.65 0.5];
    % GRF x
    options.Ix=[0.3 3 1.13];                          % [N s]
    options.grfXBetaStart=[-0.3 0 0];
    options.grfXBetaEnd=[-0.3 0 0];
    % GRF z
    options.grfZBetaStart=[0 0.5 0];
    options.grfZBetaEnd=[-0.5 0 0];

    % search and save settings
    % Adaptive phases. Counts are MultiStart starts per basic tuple and k.
    options.starts_phase1=50; options.n_phase1=7; options.k_phase1=4; options.m_phase1=5;
    options.starts_phase2=100; options.pass_phase2=50;
    options.n_phase2=9; options.m_phase2=6;
    options.starts_phase3=[200 200];
    options.pass_phase3=[25 10];
    options.n_phase3=[11 13]; options.m_phase3=[7 9];
    options.kRadiusFraction=0.10; % normalized crossing position, +/-10% of stance
    options.refinementRadius=0.20; % +/-20% of parent's search-box width
    options.resumeFolder='';       % existing adaptive run; saved settings are authoritative
    options.appendPhase3=[];       % resume only: struct('starts',200,'pass',10,'n',15,'m',11)
    options.stopAfterPhase=Inf;    % 1=phase1, 2=phase2, 3=phase3.1, ...
    options.generatePhaseOutputs=true;
    options.nodes=121;                              % broad evaluation only
    options.topResults=3;                           % global feasible top N: PNG and MP4
    options.useParallel=true;
    options.parallelWorkers=20;
    % cost function
    % J_cost: weighted squared relative errors from the speed and CoT targets.
    options.targetSpeed=4;
    options.targetCoT=.3;
    options.w1=5;
    options.w2=1;

    validateattributes(overrides,{'struct'},{'scalar'});

for name=fieldnames(overrides)', options.(name{1})=overrides.(name{1}); end
[summary,folder]=main_stage1_adaptive(options);
end
