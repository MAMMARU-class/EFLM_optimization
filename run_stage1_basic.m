function [summary,folder] = run_stage1_basic(overrides)
% Basic grid parameters: [lower upper count]; other parameters: [lower upper initial].
root=fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root,'common')),fullfile(root,'stage1_basic'));
if nargin<1, overrides=struct; end
options=struct;
options.vx0=[1.0 3.5 10];                         % [m/s], 10 grid values
options.thetaL=[deg2rad(40) deg2rad(65) 10];      % [rad], do NOT convert the count
options.r0=[.26 .33 10];                         % [m], 10 grid values
options.stanceScale=[.5 2 1];                    % continuous [lower upper initial]
options.duty=[.3 .6 .5];
options.Ix=[.1 4 1.13];                          % [N s]
options.grfXN=15;
options.grfXBetaStart=[-.5 0 0];
options.grfXBetaEnd=[-.5 0 0];
options.restrictGrfK=false;
options.grfKSelection=[7 8 9 10];                % used only if restrictGrfK=true
options.grfZN=9;
options.grfZBetaStart=[0 .5 0];
options.grfZBetaEnd=[-.5 0 0];
options.startsPerK=2000;                        % per BASIC GRID POINT and k
options.nodes=121;                              % broad evaluation only
options.topResults=3;                           % global feasible top N: PNG and MP4
options.useParallel=true;
options.parallelWorkers=20;
options.targetSpeed=4;
options.targetCoT=.3;
options.w1=5;
options.w2=1;
validateattributes(overrides,{'struct'},{'scalar'});
for name=fieldnames(overrides)', options.(name{1})=overrides.(name{1}); end
[summary,folder]=main_stage1_basic(options);
end
