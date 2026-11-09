function write_basic_rank_report(result,folder,rank,caseId)
% Human-readable final result for one exported stage1_basic rank.
filename=fullfile(folder,sprintf('report_no%d.txt',rank));
fid=fopen(filename,'w','n','UTF-8');
if fid<0, error('EFLM:ReportOpen','Could not open %s',filename); end
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
r=result.final; b=r.body; a=result.bestVariables;
cfg=result.settings; ix=cfg.index;
fprintf(fid,'Stage1 basic ranked result no%d\n',rank);
fprintf(fid,'==============================\n');
fprintf(fid,'Source case ID      : %d\n',caseId);
fprintf(fid,'Evaluation nodes    : %d\n\n',cfg.nodes);

fprintf(fid,'Final result\n------------\n');
fprintf(fid,'Result status        : %s\n',result.status);
fprintf(fid,'Final cost J_cost    : %.12g\n',r.objective);
fprintf(fid,'Basic parameters\n');
fprintf(fid,'  Initial horizontal velocity vx0 [m/s]: %.12g\n',a(ix.vx0));
fprintf(fid,'  Touchdown angle thetaL [rad]         : %.12g\n',a(ix.thetaL));
fprintf(fid,'  Initial leg length r0 [m]            : %.12g\n',a(ix.r0));
fprintf(fid,'Time parameters\n');
fprintf(fid,'  Stance time multiplier [-]           : %.12g\n',a(ix.stanceScale));
fprintf(fid,'  Duty factor [-]                      : %.12g\n',a(ix.duty));
fprintf(fid,'Horizontal GRF parameters\n');
fprintf(fid,'  Absolute impulse Ix [N s]            : %.12g\n',a(ix.Ix));
h=b.horizontalGRF;
fprintf(fid,'Horizontal GRF control points\n');
fprintf(fid,'  n=%d, k=%d\n',numel(h.knots),h.k);
fprintf(fid,'  Shape values   : %s\n',rowText(h.values));
fprintf(fid,'  Slopes [s^-1]  : %s\n',rowText(h.slopes));
fprintf(fid,'  Beta [-]       : %s\n',rowText(h.beta));
fprintf(fid,'  Impulse [N s]  : %.12g\n',h.amplitude*h.impulse);
fprintf(fid,'  Integral(f)/Ts : %.12g\n',h.normalizedImpulse);
h=b.verticalGRF;
fprintf(fid,'Vertical GRF control points\n');
fprintf(fid,'  Shape values   : %s\n',rowText(h.values));
fprintf(fid,'  Beta [-]       : %s\n',rowText(h.beta));
fprintf(fid,'Initial vertical velocity [m/s]: %.12g\n',b.vz0);
fprintf(fid,'Mean speed [m/s]     : %.12g\n',b.speed);
fprintf(fid,'CoT                  : %.12g\n',r.cot);
fprintf(fid,'Absolute work [J]    : %.12g\n',r.energy);
fprintf(fid,'Distance [m]         : %.12g\n',b.distance);
fprintf(fid,'Stance time [s]      : %.12g\n',b.Ts);
fprintf(fid,'Flight time [s]      : %.12g\n',b.Tf);
fprintf(fid,'Period [s]           : %.12g\n',b.T);
fprintf(fid,'Max violation        : %.12g\n',r.maxViolation);
fprintf(fid,'Final valid flag     : %d\n',result.valid);
idx=cfg.freeIndices;
scaled=(a(idx)-cfg.lower(idx))./(cfg.upper(idx)-cfg.lower(idx));
fprintf(fid,'Boundary distances (free variables): %s\n',rowText(min(scaled,1-scaled)));
fprintf(fid,'Periodicity residual : %s\n',rowText(b.periodic));
end

function value=rowText(x)
value=strtrim(sprintf('%.12g ',x));
end
