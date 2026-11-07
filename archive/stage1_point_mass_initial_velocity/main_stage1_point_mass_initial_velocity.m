function [result,folder] = main_stage1_point_mass_initial_velocity(overrides)
if nargin<1, overrides=struct; end
overrides.method='initial_velocity';
root=fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root,'common')));
[result,folder]=main_stage1(overrides);
end
