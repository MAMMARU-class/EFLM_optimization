function write_impulse_optimization_report(result, folder, execution, cfg, failure)
% Write one human-readable audit report for completed and failed runs.
if nargin < 5
    failure = [];
end
reportPath = fullfile(folder, 'report.txt');
fid = fopen(reportPath, 'w', 'n', 'UTF-8');
if fid < 0
    error('EFLM:ReportOpen', 'Could not open report: %s', reportPath);
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>

model='stage1_point_mass';
if isfield(cfg,'model'), model=cfg.model; end
fprintf(fid, 'EFLM %s optimization report\n',model);
fprintf(fid, '========================================\n\n');
fprintf(fid, 'Status              : %s\n', execution.status);
fprintf(fid, 'Started (Asia/Tokyo): %s\n', execution.startedAt);
fprintf(fid, 'Finished            : %s\n', execution.finishedAt);
fprintf(fid, 'Total elapsed [s]   : %.6f\n', execution.totalElapsed);
fprintf(fid, 'MATLAB              : %s\n', execution.matlabVersion);
fprintf(fid, 'Invocation directory: %s\n', execution.invocationDirectory);
fprintf(fid, 'Project root        : %s\n', execution.projectRoot);
fprintf(fid, 'Entry point         : %s\n', execution.entryPoint);
fprintf(fid, 'Call chain          : %s\n', strjoin(execution.callChain, ' -> '));
fprintf(fid, 'Git commit          : %s\n\n', execution.gitCommit);

fprintf(fid, 'Optimization route\n');
fprintf(fid, '------------------\n');
fprintf(fid, '1. Broad MultiStart -> %s/fmincon\n', cfg.algorithm);
fprintf(fid, '2. Narrow MultiStart around the broad-stage solution\n');
fprintf(fid, '3. Dense trajectory check and active-time constrained refinement\n');
fprintf(fid, '4. Save detailed trajectory, plots, video, source snapshot and this report\n\n');

fprintf(fid, 'Search settings: basic state / timing / horizontal GRF / vertical GRF\n');
fprintf(fid, '------------------------------------------------------------------\n');
if isfield(cfg,'method')
    fprintf(fid, 'Stance-time method: %s\n',cfg.method);
end
fprintf(fid, 'Lower bounds : %s\n', rowText(cfg.lower));
fprintf(fid, 'Upper bounds : %s\n', rowText(cfg.upper));
fprintf(fid, 'Initial point: %s\n', rowText(cfg.initial));
fprintf(fid, 'Variable order: %s\n',strjoin(cfg.variableNames,', '));
fprintf(fid, 'Groups: basic %s; timing %s; horizontal %s; vertical %s\n',...
 mat2str(cfg.parameterGroups.basic),mat2str(cfg.parameterGroups.timing),...
 mat2str(cfg.parameterGroups.horizontal),mat2str(cfg.parameterGroups.vertical));
fprintf(fid, 'Horizontal GRF: cubic Hermite, n=%d; sum(abs(f_i))=1; zero net impulse\n',cfg.grfXN);
fprintf(fid, 'Slope rule    : beta_i=(f_(i+1)-f_(i-1))/2; independent endpoints <= 0\n');
fprintf(fid, 'Discrete search: k=%s\n',mat2str(cfg.grfKValues));
fprintf(fid, 'Starts per k : %d\n',cfg.startsPerK);
fprintf(fid, 'Horizontal start beta [lower upper initial]: %s\n',rowText(cfg.grfXBetaStart));
fprintf(fid, 'Horizontal end beta [lower upper initial]  : %s\n',rowText(cfg.grfXBetaEnd));
fprintf(fid, 'Broad starts : %d\n', cfg.starts);
if isfield(cfg,'grfZN')
 fprintf(fid,'Vertical GRF: cubic Hermite, n=%d; interior beta from adjacent points\n',cfg.grfZN);
 fprintf(fid,'Vertical start beta [lower upper initial]: %s\n',rowText(cfg.grfZBetaStart));
 fprintf(fid,'Vertical end beta [lower upper initial]  : %s\n',rowText(cfg.grfZBetaEnd));
end
fprintf(fid, 'Narrow starts: %d\n', cfg.localStarts);
fprintf(fid, 'Narrow radius: %.6g of each scaled broad interval\n', cfg.localRadius);
fprintf(fid, 'Local solver : %s/fmincon\n', cfg.algorithm);
if isfield(cfg,'targetSpeed') && isfield(cfg,'targetCoT')
 fprintf(fid, 'Objective    : minimize w1*((meanSpeed-targetSpeed)/targetSpeed)^2 + w2*((CoT-targetCoT)/targetCoT)^2\n');
 fprintf(fid, 'Targets      : speed=%.12g m/s, CoT=%.12g\n',cfg.targetSpeed,cfg.targetCoT);
else
 fprintf(fid, 'Objective    : maximize w1*meanSpeed - w2*CoT\n');
end
fprintf(fid, 'Weights      : w1=%.12g, w2=%.12g\n', cfg.w1, cfg.w2);
fprintf(fid, 'Random seed  : %d\n', cfg.seed);
fprintf(fid, 'Stance nodes : %d (optimization), %d (final check)\n', cfg.nodes, cfg.finalNodes);
fprintf(fid, 'Minimum times: stance %.6f s, flight %.6f s\n\n', ...
    cfg.minimumStance, cfg.minimumFlight);

if ~isempty(failure)
    fprintf(fid, 'Failure\n');
    fprintf(fid, '-------\n');
    fprintf(fid, 'Identifier: %s\n', failure.identifier);
    fprintf(fid, 'Message   : %s\n', failure.message);
    for i = 1:numel(failure.stack)
        fprintf(fid, '  at %s (line %d)\n', failure.stack(i).name, failure.stack(i).line);
    end
    return
end

fprintf(fid, 'Stage statistics\n');
fprintf(fid, '----------------\n');
fprintf(fid, '%-12s %10s %10s %12s %12s %12s\n', ...
    'Stage', 'Starts', 'Success', 'Iterations', 'Evaluations', 'Elapsed[s]');
printStage(fid, 'Broad', result.stageStats.global);
printStage(fid, 'Narrow', result.stageStats.local);
printStage(fid, 'Refinement', result.stageStats.polish);
fprintf(fid, '\n');

totalOptimizationTime = result.stageStats.global.elapsed ...
    + result.stageStats.local.elapsed + result.stageStats.polish.elapsed;
totalOptimizationIterations = result.stageStats.global.iterations ...
    + result.stageStats.local.iterations + result.stageStats.polish.iterations;
if totalOptimizationIterations > 0
    averageIterationTime = totalOptimizationTime / totalOptimizationIterations;
else
    averageIterationTime = NaN;
end
fprintf(fid, 'Model processing time\n');
fprintf(fid, '---------------------\n');
fprintf(fid, 'Total optimization time [s]       : %.6f\n', totalOptimizationTime);
fprintf(fid, 'Total optimization iterations     : %d\n', totalOptimizationIterations);
fprintf(fid, 'Average time per iteration [s]    : %.9f\n\n', averageIterationTime);

r = result.final;
b = r.body;
a = result.bestVariables;
fprintf(fid, 'Final result\n');
fprintf(fid, '------------\n');
fprintf(fid, 'Result status        : %s\n', result.status);
fprintf(fid, 'Final cost J_cost    : %.12g\n', r.objective);
ix=result.settings.index;
fprintf(fid, 'Basic parameters\n');
fprintf(fid, '  Initial horizontal velocity vx0 [m/s]: %.12g\n',a(ix.vx0));
fprintf(fid, '  Touchdown angle thetaL [rad]         : %.12g\n',a(ix.thetaL));
fprintf(fid, '  Initial leg length r0 [m]            : %.12g\n',a(ix.r0));
fprintf(fid, 'Time parameters\n');
fprintf(fid, '  Stance time multiplier [-]           : %.12g\n',a(ix.stanceScale));
fprintf(fid, '  Duty factor [-]                      : %.12g\n',a(ix.duty));
fprintf(fid, 'Horizontal GRF parameters\n');
fprintf(fid, '  Absolute impulse Ix [N s]            : %.12g\n',a(ix.Ix));
if isfield(b,'horizontalGRF')
 h=b.horizontalGRF;
 fprintf(fid, 'Horizontal GRF control points\n');
 fprintf(fid, '  n=%d, k=%d\n',numel(h.knots),h.k);
 fprintf(fid, '  Shape values   : %s\n',rowText(h.values));
 fprintf(fid, '  Slopes [s^-1]  : %s\n',rowText(h.slopes));
 if isfield(h,'beta'), fprintf(fid, '  Beta [-]       : %s\n',rowText(h.beta)); end
 fprintf(fid, '  Impulse [N s]  : %.12g\n',h.amplitude*h.impulse);
 fprintf(fid, '  Integral(f)/Ts : %.12g\n',h.normalizedImpulse);
end
if isfield(b,'verticalGRF')
 h=b.verticalGRF;
 fprintf(fid,'Vertical GRF control points\n');
 fprintf(fid,'  Shape values   : %s\n',rowText(h.values));
 fprintf(fid,'  Beta [-]       : %s\n',rowText(h.beta));
 fprintf(fid,'Initial vertical velocity [m/s]: %.12g\n',b.vz0);
end
fprintf(fid, 'Mean speed [m/s]     : %.12g\n', b.speed);
fprintf(fid, 'CoT                  : %.12g\n', r.cot);
fprintf(fid, 'Absolute work [J]    : %.12g\n', r.energy);
fprintf(fid, 'Distance [m]         : %.12g\n', b.distance);
fprintf(fid, 'Stance time [s]      : %.12g\n', b.Ts);
fprintf(fid, 'Flight time [s]      : %.12g\n', b.Tf);
fprintf(fid, 'Period [s]           : %.12g\n', b.T);
fprintf(fid, 'Max violation        : %.12g\n', r.maxViolation);
fprintf(fid, 'Final valid flag     : %d\n', result.valid);
fprintf(fid, 'Boundary distances   : %s\n', rowText(result.boundaryDistance));
fprintf(fid, 'Periodicity residual : %s\n\n', rowText(b.periodic));

fprintf(fid, 'Artifacts\n');
fprintf(fid, '---------\n');
fprintf(fid, 'result.mat, summary.csv, trajectory.csv,\n');
if isfield(b,'verticalGRF')
 fprintf(fid,'vertical_grf_control_points.csv, vertical_grf_shape_constraints.csv\n');
end
fprintf(fid, 'constraint_residuals.csv, plots/, animation/, source/, settings.mat\n');
if isfield(b,'horizontalGRF')
 fprintf(fid, 'horizontal_grf_control_points.csv, horizontal_grf_shape_constraints.csv, horizontal_grf_summary.csv\n');
 if isfield(b.horizontalGRF,'beta'), fprintf(fid,'horizontal_grf_beta.csv\n'); end
end
end

function printStage(fid, name, stats)
fprintf(fid, '%-12s %10d %10d %12d %12d %12.6f\n', name, ...
    stats.starts, stats.successfulStarts, stats.iterations, ...
    stats.functionEvaluations, stats.elapsed);
end

function value = rowText(x)
value = strtrim(sprintf('%.12g ', x));
end
