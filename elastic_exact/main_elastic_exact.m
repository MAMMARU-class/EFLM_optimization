function [result,folder] = main_elastic_exact(overrides)
if nargin<1, overrides=struct; end
root=fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root,'common')));
[result,folder]=main_elastic('exact',overrides);
end
