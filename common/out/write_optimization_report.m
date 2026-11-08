function write_optimization_report(result, folder, execution, cfg, failure)
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

fprintf(fid, 'Search settings (AmpH [N], thetaL [rad], v [m/s], Duty [-], r0 [m]');
if numel(cfg.initial)>=6, fprintf(fid, ', stanceScale [-]'); end
if isfield(cfg,'grfN'), fprintf(fid, ', Hermite ratios [-] and slopes [s^-1]'); end
fprintf(fid, ')\n');
fprintf(fid, '---------------------------------------------------------------\n');
if isfield(cfg,'method')
    fprintf(fid, 'Stance-time method: %s\n',cfg.method);
end
fprintf(fid, 'Lower bounds : %s\n', rowText(cfg.lower));
fprintf(fid, 'Upper bounds : %s\n', rowText(cfg.upper));
fprintf(fid, 'Initial point: %s\n', rowText(cfg.initial));
if isfield(cfg,'grfN')
 fprintf(fid, 'Variable order: %s\n',strjoin(cfg.variableNames,', '));
 fprintf(fid, 'Horizontal GRF: cubic Hermite, n=%d, slopes [s^-1]\n',cfg.grfN);
 fprintf(fid, 'Slope rule    : interior central differences, b_j=0; only b_1<0 and b_n<0 are free\n');
 fprintf(fid, 'Discrete search: %d (k,j) pairs share the broad starts\n',size(cfg.grfPairs,1));
end
fprintf(fid, 'Broad starts : %d\n', cfg.starts);
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
fprintf(fid, 'Optimized initial parameters\n');
fprintf(fid, '  Horizontal force amplitude AmpH [N]  : %.12g\n', a(1));
fprintf(fid, '  Touchdown angle thetaL [rad]         : %.12g\n', a(2));
fprintf(fid, '  Initial horizontal velocity v [m/s]  : %.12g\n', a(3));
fprintf(fid, '  Duty factor [-]                      : %.12g\n', a(4));
fprintf(fid, '  Initial leg length r0 [m]            : %.12g\n', a(5));
if numel(a)>=6
 fprintf(fid, '  Stance time multiplier [-]          : %.12g\n',a(6));
end
if isfield(b,'horizontalGRF')
 h=b.horizontalGRF;
 fprintf(fid, 'Horizontal GRF control points\n');
 fprintf(fid, '  n=%d, k=%d, j=%d\n',numel(h.knots),h.k,h.j);
 fprintf(fid, '  Shape values   : %s\n',rowText(h.values));
 fprintf(fid, '  Slopes [s^-1]  : %s\n',rowText(h.slopes));
 fprintf(fid, '  Impulse [N s]  : %.12g\n',a(1)*h.impulse);
 fprintf(fid, '  Integral(f)/Ts : %.12g\n',h.normalizedImpulse);
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
fprintf(fid, 'constraint_residuals.csv, plots/, animation/, source/, settings.mat\n');
if isfield(b,'horizontalGRF')
 fprintf(fid, 'horizontal_grf_control_points.csv, horizontal_grf_shape_constraints.csv, horizontal_grf_summary.csv\n');
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
