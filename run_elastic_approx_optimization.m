function [result,folder] = run_elastic_approx_optimization(varargin)
% Compatibility alias. Also accepts 'exact'; edit presets in run_elastic_optimization.m.
[result,folder]=run_elastic_optimization(varargin{:});
end
