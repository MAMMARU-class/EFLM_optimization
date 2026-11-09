function cleanup = suppress_basic_rank_warning(cfg)
% Scope this one warning setting to the grid run, including process workers.
id='MATLAB:rankDeficientMatrix';
previous=warning('off',id);
workerState=[];
if cfg.useParallel
 % Each worker remembers its own original setting and restores it on deletion.
 workerState=parallel.pool.Constant(@()warning('off',id),@(state)warning(state));
end
cleanup=onCleanup(@restore);
if cfg.useParallel
 f=parfevalOnAll(gcp('nocreate'),@initializeWarning,0,workerState);
 wait(f);
 for i=1:numel(f)
  if ~isempty(f(i).Error), throw(f(i).Error); end
 end
end
 function restore()
  warning(previous);
  if ~isempty(workerState), delete(workerState); end
 end
end

function initializeWarning(state)
previous=state.Value; %#ok<NASGU> % Initialize the setting on this worker.
end
