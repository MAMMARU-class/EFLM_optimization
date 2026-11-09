function [outputFolder,sourceFolder] = plot_stage1_basic_parameter_maps(sourceFolder)
% Recreate the stage1_basic parameter maps under _logs/test/<timestamp>.
root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'stage1_basic'));
if nargin<1
 [outputFolder,sourceFolder]=plot_basic_grid_search_results();
else
 [outputFolder,sourceFolder]=plot_basic_grid_search_results(sourceFolder);
end
end
