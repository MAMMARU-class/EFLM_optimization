function recreate_outputs(resultFile,makeVideo)
% Regenerate outputs without rerunning the optimizer.
if nargin<2, makeVideo=true; end
loaded=load(resultFile,'result'); result=loaded.result;
folder=fileparts(resultFile);
save_run_outputs(result,folder);
if makeVideo, render_eflm_video(result,folder); end
end
