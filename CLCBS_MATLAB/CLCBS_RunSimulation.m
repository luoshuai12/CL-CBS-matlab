function results = CLCBS_RunSimulation(cfg)
%CLCBS_RUNSIMULATION Unique GUI-safe entry point for the CL-CBS MATLAB demo.
%   This avoids collisions with other user/project functions named main,
%   CreateMap, CBS, etc. by activating this project path first.

    rootDir = fileparts(mfilename('fullpath'));
    activateProjectPath(rootDir);

    if nargin < 1 || isempty(cfg)
        cfg = struct();
    end
    cfg = applyDefaults(cfg, rootDir);

    createConfig = struct();
    createConfig.mapSize = cfg.mapSize;
    createConfig.obstacleCount = cfg.obstacleCount;
    createConfig.randomSeed = cfg.randomSeed;
    createConfig.params = cfg.params;

    [mapData, starts, goals, params] = CreateMap(createConfig);
    params = mergeStructLocal(params, cfg.params);
    params.xyResolution = params.r * params.deltat;
    params.yawResolution = params.deltat;

    if ~exist(cfg.outputDir, 'dir')
        mkdir(cfg.outputDir);
    end

    tic;
    [solution, success, stats] = CBS(mapData, starts, goals, params);
    stats.runtime = toc;

    results = struct();
    results.map = mapData;
    results.starts = starts;
    results.goals = goals;
    results.params = params;
    results.solution = solution;
    results.success = success;
    results.stats = stats;
    results.outputDir = cfg.outputDir;

    if success
        results.error = ComputeError(solution, starts, 1, 2:size(starts, 1));
        saveResults(results);
    else
        results.error = ComputeError({}, starts, 1, []);
    end
end

function activateProjectPath(rootDir)
    addpath(rootDir, '-begin');
    addpath(fullfile(rootDir, 'map'), '-begin');
    addpath(fullfile(rootDir, 'high_level'), '-begin');
    addpath(fullfile(rootDir, 'low_level'), '-begin');
    addpath(fullfile(rootDir, 'utils'), '-begin');
    clear CreateMap CBS HybridAstar DubinsHeuristic DubinsPlanner CheckConstraint;
    clear DetectConflict GenerateChild ComputeCost ComputeError CollisionCheck;
    clear DrawMap DrawTrajectory DrawVehicle InterpolatePath;
    rehash;
end

function cfg = applyDefaults(cfg, rootDir)
    if ~isfield(cfg, 'mapSize')
        cfg.mapSize = [150, 50];
    end
    if ~isfield(cfg, 'obstacleCount')
        cfg.obstacleCount = 10;
    end
    if ~isfield(cfg, 'randomSeed')
        cfg.randomSeed = 20260602 + cfg.obstacleCount;
    end
    if ~isfield(cfg, 'outputDir')
        cfg.outputDir = fullfile(rootDir, 'result', 'save_results');
    end
    if ~isfield(cfg, 'params')
        cfg.params = struct();
    end
end

function saveResults(results)
    if ~isfield(results, 'outputDir') || ~exist(results.outputDir, 'dir')
        return;
    end

    save(fullfile(results.outputDir, 'clcbs_results.mat'), 'results');
    statsRow = [results.success, results.stats.cost, results.stats.makespan, ...
        results.stats.highLevelExpanded, results.stats.lowLevelExpanded, ...
        results.stats.conflictsResolved, results.stats.runtime, ...
        results.error.meanError, results.error.maxError];
    writeMatrixLocal(fullfile(results.outputDir, 'stats_summary.csv'), statsRow);

    for i = 1:numel(results.solution)
        writeMatrixLocal(fullfile(results.outputDir, sprintf('agent_%02d_path.csv', i)), ...
            results.solution{i}.states);
    end
    writeMatrixLocal(fullfile(results.outputDir, 'formation_error.csv'), ...
        [results.error.time, results.error.errors]);
end

function writeMatrixLocal(filename, data)
    try
        writematrix(data, filename);
    catch
        csvwrite(filename, data);
    end
end

function merged = mergeStructLocal(base, patch)
    merged = base;
    if isempty(patch)
        return;
    end
    names = fieldnames(patch);
    for i = 1:numel(names)
        f = names{i};
        if isstruct(patch.(f)) && isfield(merged, f) && isstruct(merged.(f))
            merged.(f) = mergeStructLocal(merged.(f), patch.(f));
        else
            merged.(f) = patch.(f);
        end
    end
end
