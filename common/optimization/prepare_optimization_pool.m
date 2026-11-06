function prepare_optimization_pool(cfg)
if ~cfg.useParallel, return; end
validateattributes(cfg.parallelWorkers,{'numeric'},{'scalar','integer','positive','finite'});
if exist('parpool','file')~=2 || ~license('test','Distrib_Computing_Toolbox')
 error('EFLM:MissingParallelToolbox',...
  'Parallel Computing Toolbox is required for the configured MultiStart parallel run. Install it in R2026b.');
end
pool=gcp('nocreate');
if isempty(pool)
 pool=parpool('Processes',cfg.parallelWorkers);
elseif ~isa(pool,'parallel.ProcessPool') || pool.NumWorkers~=cfg.parallelWorkers
 fprintf('Replacing existing %s pool (%d workers) with a %d-worker process pool.\n',...
  class(pool),pool.NumWorkers,cfg.parallelWorkers);
 delete(pool);
 pool=parpool('Processes',cfg.parallelWorkers);
end
% Existing pools must also receive the current project path.
f=parfevalOnAll(pool,@path,0,path); wait(f);
for i=1:numel(f)
 if ~isempty(f(i).Error), throw(f(i).Error); end
end
fprintf('MultiStart: %d process workers\n',pool.NumWorkers);
end
