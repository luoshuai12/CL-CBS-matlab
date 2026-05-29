function results = run_cl_cbs_formation_validation(userConfig)
% run_cl_cbs_formation_validation
% Compatibility wrapper.
%
% New modular implementation is under:
%   matlab/clcbs/Main_CLCBS_Validation.m
%
% This wrapper only runs CL-CBS pipeline now.

    if nargin < 1
        userConfig = struct();
    end

    warning('run_cl_cbs_formation_validation 已切换为多文件实现，建议直接调用 clcbs/Main_CLCBS_Validation。');

    thisDir = fileparts(mfilename('fullpath'));
    addpath(fullfile(thisDir, 'clcbs'));
    results = Main_CLCBS_Validation(userConfig);
end
