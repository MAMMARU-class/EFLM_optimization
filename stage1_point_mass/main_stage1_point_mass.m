function [result,folder] = main_stage1_point_mass(overrides)
if nargin<1, overrides=struct; end
root=fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root,'common')));
[result,folder]=main_stage1(overrides);
end
