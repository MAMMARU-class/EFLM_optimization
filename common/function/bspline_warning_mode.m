function bspline_warning_mode(action,cfg)
% Temporarily silence MATLAB warnings on the client and process workers.
persistent previousState
switch action
 case 'disable'
  previousState=warning('query','all');
  warning('off','all');
 case 'restore'
  if ~isempty(previousState), warning(previousState); previousState=[]; end
 otherwise
  error('EFLM:WarningMode','Unknown warning mode: %s',action);
end
if cfg.useParallel
 pool=gcp('nocreate');
 if ~isempty(pool)
  try
   tasks=parfevalOnAll(pool,@bspline_worker_warning,0,action); wait(tasks);
   if strcmp(action,'disable')
    for i=1:numel(tasks)
     if ~isempty(tasks(i).Error), throw(tasks(i).Error); end
    end
   end
  catch ME
   if strcmp(action,'disable'), rethrow(ME); end
   % A failed pool cannot have its worker warning state restored.
  end
 end
end
end
