function [result, folder] = run_stage1_bspline(overrides)
clc
% Compare cubic B-spline GRFs with Hermite run 20261109_165106_197.
% Usage: run_stage1_bspline() or run_stage1_bspline(options).
% Basic/timing parameters are [lower, upper, initial].
% The first and last B-spline coefficients are fixed at zero.

if nargin < 1
    overrides = struct;

    % Basic initial state: same bounds and initial values as the Hermite run.
    overrides.vx0 = [1.0, 3.5, 1.0];              % initial horizontal speed [m/s]
    overrides.thetaL = deg2rad([40, 65, 50]);      % touchdown angle [rad]
    overrides.r0 = [0.26, 0.33, 0.30];            % initial leg length [m]

    % Timing. The minimum stance/flight durations remain 0.2 s each.
    overrides.stanceScale = [0.5, 2.0, 1.0];       % stance-time multiplier [-]
    overrides.duty = [0.3, 0.60, 0.50];           % duty factor [-]

    % Cubic open-uniform B-splines in x and z.
    overrides.bsplineXN = 16;                    % x control points, including two fixed zeros
    overrides.bsplineZN = 11;                    % z control points, including two fixed zeros
    overrides.bsplineForceBound = 100;           % each free x coefficient in [-100, 100] N

    % Search effort: Hermite used 11 k values * 2000 starts = 22000 starts.
    overrides.broadStarts = 220000;
    overrides.localStarts = 10000;
    overrides.save_interval = 1000;                % completed starts per checkpoint
    overrides.resumeFolder = '';                  % existing B-spline run folder
    overrides.useParallel = true;
    overrides.parallelWorkers = 20;
    overrides.nodes = 121;                       % broad and narrow evaluation
    overrides.finalNodes = 1201;                 % final refinement and check
    overrides.localRadius = 0.15;                % narrow box radius, normalized
    overrides.seed = 20261103;
    overrides.algorithm = 'sqp-legacy';
    overrides.maxIterations = 120;
    overrides.maxEvaluations = 1400;
    overrides.constraintTolerance = 1e-5;
    overrides.stepTolerance = 1e-7;
    overrides.minimumStance = 0.2;               % [s]
    overrides.minimumFlight = 0.2;               % [s]
    overrides.video = true;

    % Weighted squared relative errors from the targets.
    overrides.targetSpeed = 4;                   % cycle-mean speed [m/s]
    overrides.targetCoT = 0.3;
    overrides.w1 = 5;
    overrides.w2 = 1;
end
validateattributes(overrides, {'struct'}, {'scalar'}, mfilename, 'overrides');

root=fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root,'common')),fullfile(root,'stage1_point_mass'));
resumeFolder='';
if isfield(overrides,'resumeFolder')
 resumeFolder=char(overrides.resumeFolder);
 overrides=rmfield(overrides,'resumeFolder');
end
if ~isempty(resumeFolder)
 if ~isempty(fieldnames(overrides))
  error('EFLM:ResumeSettings','Pass only resumeFolder; saved settings are authoritative.');
 end
 saved=load(fullfile(resumeFolder,'settings.mat'),'cfg','p');
 cfg=saved.cfg; p=saved.p;
 if ~isfield(cfg,'save_interval') || cfg.layoutVersion~=3
  error('EFLM:ResumeSettings','This run has no resumable B-spline checkpoints.');
 end
 folder=resumeFolder;
 fprintf('Resuming %s with its saved settings.\n',folder);
else
cfg=optimization_settings(); cfg.layoutVersion=3;
cfg=rmfield(cfg,{'startsPerK','restrictGrfK','grfKSelection','grfXN','grfZN',...
 'grfXBetaStart','grfXBetaEnd','grfZBetaStart','grfZBetaEnd'});
cfg.lower=cfg.lower(1:5);
cfg.upper=cfg.upper(1:5);
cfg.bsplineXN=16;
cfg.bsplineZN=11;
cfg.bsplineForceBound=100;
cfg.broadStarts=22000;
cfg.localStarts=1000;
cfg.save_interval=100;
cfg.parallelWorkers=20;
cfg.w1=5;
cfg.w2=1;
cfg.targetSpeed=4;
cfg.targetCoT=.3;
cfg.initial=[1 deg2rad(50) .30 1 .5];
cfg.method='direct_force_open_uniform_cubic_bspline'; cfg.model='stage1_bspline';
cfg.index=struct('r0',3,'stanceScale',4);
basicNames={'vx0','thetaL','r0','stanceScale','duty'};
for i=1:numel(basicNames)
 name=basicNames{i};
 if ~isfield(overrides,name), continue; end
 values=overrides.(name);
 validateattributes(values,{'numeric'},{'row','numel',3,'real','finite'},mfilename,name);
 cfg.lower(i)=values(1); cfg.upper(i)=values(2); cfg.initial(i)=values(3);
 overrides=rmfield(overrides,name);
end
for name=fieldnames(overrides)'
 if ~isfield(cfg,name{1}), error('EFLM:UnknownSetting','Unknown setting: %s',name{1}); end
 cfg.(name{1})=overrides.(name{1});
end
validateattributes(cfg.bsplineXN,{'numeric'},{'scalar','integer','>=',4});
validateattributes(cfg.bsplineZN,{'numeric'},{'scalar','integer','>=',4});
validateattributes(cfg.bsplineForceBound,{'numeric'},{'scalar','finite','positive'});
validateattributes(cfg.broadStarts,{'numeric'},{'scalar','integer','positive'});
validateattributes(cfg.localStarts,{'numeric'},{'scalar','integer','positive'});
validateattributes(cfg.save_interval,{'numeric'},{'scalar','integer','positive'});
validateattributes(cfg.nodes,{'numeric'},{'scalar','integer','>=',3});
validateattributes(cfg.finalNodes,{'numeric'},{'scalar','integer','>=',3});
validateattributes(cfg.lower,{'numeric'},{'row','numel',5,'real','finite'});
validateattributes(cfg.upper,{'numeric'},{'row','numel',5,'real','finite'});
validateattributes(cfg.initial,{'numeric'},{'row','numel',5,'real','finite'});
if any(cfg.lower>=cfg.upper) || any(cfg.initial<cfg.lower | cfg.initial>cfg.upper) || ...
 cfg.lower(1)<=0 || cfg.lower(3)<=0 || cfg.lower(4)<=0 || cfg.lower(5)<=0 || cfg.upper(5)>=1
 error('EFLM:Bounds','Invalid basic/timing bounds or initial point.');
end
nx=cfg.bsplineXN-2; nz=cfg.bsplineZN-2;
cfg.index.fx=6:5+nx; cfg.index.fz=6+nx:5+nx+nz;
cfg.variableNames={'vx0','thetaL','r0','stanceScale','duty'};
for i=1:nx, cfg.variableNames{end+1}=sprintf('FxControl%d',i+1); end
for i=1:nz, cfg.variableNames{end+1}=sprintf('FzShapeControl%d',i+1); end
cfg.lower=[cfg.lower,-cfg.bsplineForceBound*ones(1,nx),zeros(1,nz)];
cfg.upper=[cfg.upper,cfg.bsplineForceBound*ones(1,nx),ones(1,nz)];
cfg.initial=[cfg.initial,linspace(-10,10,nx),ones(1,nz)/nz];
p=robot_parameters();
folder=fullfile(root,'_logs','stage1_bspline',char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'))); mkdir(folder);
save(fullfile(folder,'settings.mat'),'cfg','p');
copyfile(fullfile(root,'common'),fullfile(folder,'source','common'));
copyfile(fullfile(root,'stage1_point_mass'),fullfile(folder,'source','stage1_point_mass'));
copyfile(fullfile(root,'run_stage1_bspline.m'),fullfile(folder,'source','run_stage1_bspline.m'));
end
tab=build_exact_geometry_cache(p);
prepare_optimization_pool(cfg);
warningGuard=onCleanup(@()bspline_warning_mode('restore',cfg)); %#ok<NASGU>
try
 bspline_warning_mode('disable',cfg);
 result=optimize_bspline_two_stage(p,tab,cfg,folder);
catch ME
 fid=fopen(fullfile(folder,'report.txt'),'w');
 fprintf(fid,'B-spline optimization failed\n%s\n',getReport(ME,'extended')); fclose(fid);
 fprintf('Optimization stopped. Details: %s\n',fullfile(folder,'report.txt'));
 fprintf('Resume: run_stage1_bspline(struct(''resumeFolder'',''%s''))\n',strrep(folder,'''',''''''));
 result=[];
 return;
end
sx=result.final.body.bspline;
result.final.body.bspline.absoluteImpulse=result.final.body.Ts*bspline_absolute_area(sx.cx,sx.knotsX);
save(fullfile(folder,'result.mat'),'result','-v7');
% Generic trajectory outputs do not interpret B-spline coefficients as samples.
export=result; export.final.body=rmfield(export.final.body,{'horizontalGRF','verticalGRF'});
save_run_outputs(export,folder);
for axis={'x','z'}
 label=axis{1}; c=result.final.body.bspline.(['c' label]);
 amplitude=result.final.body.bspline.(['A' label]);
 writetable(table((1:numel(c))',c,amplitude*c,'VariableNames',{'controlIndex','coefficient','forceCoefficientN'}),...
  fullfile(folder,['bspline_' label '_controls.csv']));
end
if cfg.video, render_eflm_video(result,folder); end
fid=fopen(fullfile(folder,'report.txt'),'w'); guard=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'Open-uniform cubic B-spline; reference 20261109_165106_197\n');
fprintf(fid,'Variables: %d; independent equality constraints: 2; effective dimension: %d\n',numel(cfg.initial),numel(cfg.initial)-2);
fprintf(fid,'Horizontal coefficient bounds: +/- %.9g N\n',cfg.bsplineForceBound);
fprintf(fid,'Broad starts: %d; narrow starts: %d; broad/narrow nodes: %d; final nodes: %d\n',cfg.broadStarts,cfg.localStarts,cfg.nodes,cfg.finalNodes);
fprintf(fid,'Status: %s\nCost: %.12g\nSpeed: %.12g\nCoT: %.12g\nMax violation: %.12g\n',...
 result.status,result.final.objective,result.final.body.speed,result.final.cot,result.final.maxViolation);
fprintf(fid,'Absolute horizontal impulse (postprocessed): %.12g N s\n',result.final.body.bspline.absoluteImpulse);
fprintf(fid,'Maximum absolute horizontal coefficient: %.12g N\n',max(abs(sx.cx)));
fprintf('Saved: %s\n',folder);
end
