function write_basic_report(folder,summary,cfg,meta)
fid=fopen(fullfile(folder,'report.txt'),'w','n','UTF-8');
if fid<0, error('EFLM:ReportOpen','Cannot write report.txt'); end
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'Stage1 basic grid search\n========================\n');
fprintf(fid,'Status: %s\nStarted: %s\nExecution time [s]: %.6f\nMATLAB: %s\nGit: %s\n',...
 meta.status,meta.startedAt,meta.elapsed,meta.matlabVersion,meta.gitCommit);
fprintf(fid,'Completed combinations: %d / %d\n',height(summary),meta.totalCases);
fprintf(fid,'Route: basic grid -> each k -> broad MultiStart only\n');
fprintf(fid,'Grid [center half-width count]\n');
names={'vx0 [m/s]','thetaL [rad]','r0 [m]'};
for i=1:3, fprintf(fid,'  %-14s: %.12g %.12g %d\n',names{i},cfg.gridSpecs(i,:)); end
fprintf(fid,'k: %s\nStarts per grid point and k: %d\nNodes: %d\n',mat2str(cfg.grfKValues),cfg.startsPerK,cfg.nodes);
fprintf(fid,'Variable order: %s\n',strjoin(cfg.variableNames,', '));
fprintf(fid,'Continuous lower: %s\nContinuous upper: %s\n',mat2str(cfg.lower(cfg.freeIndices),12),mat2str(cfg.upper(cfg.freeIndices),12));
fprintf(fid,'Target speed: %.12g; target CoT: %.12g; weights: %.12g %.12g\n',cfg.targetSpeed,cfg.targetCoT,cfg.w1,cfg.w2);
fprintf(fid,'Feasibility: max constraint violation <= %.12g at %d nodes (no dense refinement).\n',cfg.constraintTolerance,cfg.nodes);
fprintf(fid,'Failure to find a feasible candidate is not proof of infeasibility.\n');
fprintf(fid,'Every completed combination is saved in cases/ and summary.csv. No solver history is stored.\n');
if ~isempty(summary)
 fprintf(fid,'Feasible combinations: %d\n',nnz(summary.feasible));
 fprintf(fid,'Total iterations: %.0f\n',sum(summary.iterations));
 ranked=sortrows(summary(summary.feasible,:),{'cost','caseId'});
 count=min(cfg.topResults,height(ranked));
 fprintf(fid,'\nTop %d feasible combinations (cost ascending)\n',count);
 for i=1:count
  a=ranked(i,:);
  fprintf(fid,['no%d: case=%d grid=%d k=%d cost=%.12g meanSpeed=%.12g CoT=%.12g ' ...
   'vx0=%.12g thetaL=%.12g r0=%.12g\n'],...
   i,a.caseId,a.gridId,a.k,a.cost,a.speed,a.CoT,a.vx0,a.thetaL,a.r0);
 end
end
if isfield(meta,'error'), fprintf(fid,'\nError: %s\n',meta.error); end
end
