function bspline_worker_warning(action)
% Runs separately on every process worker; retains its original warning state.
persistent previousState
switch action
 case 'disable'
  if isempty(previousState), previousState=warning('query','all'); end
  warning('off','all');
 case 'restore'
  if ~isempty(previousState), warning(previousState); previousState=[]; end
end
end
