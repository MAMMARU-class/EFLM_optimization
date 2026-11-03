function [result,folder] = main_elastic_approx(overrides)
% Backward-compatible approx entry point.
if nargin<1, overrides=struct; end
root=fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root,'common')));
[result,folder]=main_elastic('approx',overrides);
end
