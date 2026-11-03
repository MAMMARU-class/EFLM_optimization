function [result,folder] = main_EFLM(overrides)
if nargin<1, overrides=struct; end
root=fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root,'common')));
[result,folder]=main_elastic('EFLM',overrides);
end
