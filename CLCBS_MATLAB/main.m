function results = main(userConfig)
%MAIN Entry point for the MATLAB CL-CBS port.
%   Run from this directory with:
%       results = main();
%
%   Optional:
%       cfg.showFigure = false;
%       cfg.saveResults = true;
%       cfg.params.maxHighLevelIterations = 120;
%       results = main(cfg);

    if nargin < 1
        userConfig = struct();
    end

    rootDir = fileparts(mfilename('fullpath'));
    addpath(fullfile(rootDir, 'map'));
    addpath(fullfile(rootDir, 'high_level'));
    addpath(fullfile(rootDir, 'low_level'));
    addpath(fullfile(rootDir, 'utils'));

    cfg = defaultRunConfig(rootDir);
    cfg = mergeStruct(cfg, userConfig);

    if cfg.forceGenerate || ~exist(cfg.scenarioFile, 'file') || ~exist(cfg.mapFile, 'file')
        createConfig = struct();
        createConfig.mapSize = cfg.mapSize;
        createConfig.obstacleCount = cfg.obstacleCount;
        createConfig.randomSeed = cfg.randomSeed;
        if isfield(cfg, 'params')
            createConfig.params = cfg.params;
        end
        [mapData, starts, goals, params] = CreateMap(createConfig);
        saveMap(cfg.mapFile, mapData);
        saveScenario(cfg.scenarioFile, starts, goals, params);
    else
        [mapData, starts, goals, params] = loadScenario(cfg.scenarioFile, cfg.mapFile);
    end

    if isfield(cfg, 'params')
        params = mergeStruct(params, cfg.params);
        params.xyResolution = params.r * params.deltat;
        params.yawResolution = params.deltat;
    end

    if ~exist(cfg.outputDir, 'dir')
        mkdir(cfg.outputDir);
    end

    fprintf('== CL-CBS MATLAB demo ==\n');
    fprintf('Map: %.1f x %.1f m, agents: %d, obstacles: %d\n', ...
        mapData.size(1), mapData.size(2), size(starts, 1), size(mapData.obstacles, 1));

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
        leaderIdx = pickField(cfg, 'leaderIdx', 1);
        followerIdx = pickField(cfg, 'followerIdx', setdiff(1:size(starts, 1), leaderIdx));
        results.error = ComputeError(solution, starts, leaderIdx, followerIdx);
    else
        results.error = ComputeError({}, starts, 1, []);
    end

    if success
        fprintf('Success. cost=%.3f, makespan=%d, high-level expanded=%d, runtime=%.3fs\n', ...
            stats.cost, stats.makespan, stats.highLevelExpanded, stats.runtime);
    else
        fprintf('Failed to find collision-free paths. high-level expanded=%d, runtime=%.3fs\n', ...
            stats.highLevelExpanded, stats.runtime);
    end

    fig = [];
    if cfg.showFigure
        fig = figure('Name', 'CL-CBS MATLAB');
        DrawMap(mapData, starts, goals, params);
        if success
            DrawTrajectory(solution, params);
        end
        title('Car-Like Conflict-Based Search (MATLAB port)');
    end

    if cfg.saveResults
        saveResults(results, fig, cfg);
    end
end

function cfg = defaultRunConfig(rootDir)
    cfg = struct();
    cfg.dataDir = fullfile(rootDir, 'data');
    cfg.mapFile = fullfile(cfg.dataDir, 'map1.mat');
    cfg.scenarioFile = fullfile(cfg.dataDir, 'scenario1.mat');
    cfg.outputDir = fullfile(rootDir, 'result', 'save_results');
    cfg.showFigure = true;
    cfg.saveResults = true;
    cfg.forceGenerate = true;
    cfg.mapSize = [150, 50];
    cfg.obstacleCount = 10;
    cfg.randomSeed = 20260602;
    cfg.params = struct();
    cfg.params.maxHighLevelIterations = 240;
    cfg.params.maxLowLevelNodes = 90000;
end

function [mapData, starts, goals, params] = loadScenario(scenarioFile, mapFile)
    mapRaw = load(mapFile);
    if isfield(mapRaw, 'mapData')
        mapData = mapRaw.mapData;
    else
        mapData = struct();
        mapData.size = mapRaw.mapSize(:).';
        mapData.obstacles = mapRaw.obstacles;
        mapData.dynamicObstacles = [];
    end

    raw = load(scenarioFile);
    starts = raw.starts;
    goals = raw.goals;

    emptyConfig = struct();
    emptyConfig.obstacles = zeros(0, 3);
    [~, ~, ~, params] = CreateMap(emptyConfig);
    if isfield(raw, 'paramsVector')
        params = unpackParams(raw.paramsVector, params);
    end
end

function saveMap(filename, mapData)
    outDir = fileparts(filename);
    if ~exist(outDir, 'dir')
        mkdir(outDir);
    end
    mapSize = mapData.size; %#ok<NASGU>
    obstacles = mapData.obstacles; %#ok<NASGU>
    save(filename, 'mapSize', 'obstacles', 'mapData');
end

function saveScenario(filename, starts, goals, params)
    outDir = fileparts(filename);
    if ~exist(outDir, 'dir')
        mkdir(outDir);
    end
    paramsVector = packParams(params); %#ok<NASGU>
    save(filename, 'starts', 'goals', 'paramsVector');
end

function saveResults(results, fig, cfg)
    if ~exist(cfg.outputDir, 'dir')
        mkdir(cfg.outputDir);
    end

    save(fullfile(cfg.outputDir, 'clcbs_results.mat'), 'results');

    statsRow = [results.success, results.stats.cost, results.stats.makespan, ...
        results.stats.highLevelExpanded, results.stats.lowLevelExpanded, ...
        results.stats.conflictsResolved, results.stats.runtime, ...
        results.error.meanError, results.error.maxError];
    writeMatrixCompat(fullfile(cfg.outputDir, 'stats_summary.csv'), statsRow);

    if results.success
        for i = 1:numel(results.solution)
            pathData = results.solution{i}.states;
            writeMatrixCompat(fullfile(cfg.outputDir, sprintf('agent_%02d_path.csv', i)), pathData);
        end
        errorData = [results.error.time, results.error.errors];
        writeMatrixCompat(fullfile(cfg.outputDir, 'formation_error.csv'), errorData);
        saveErrorFigure(results, cfg.outputDir);
        saveFigure(results, fig, cfg.outputDir);
    end
end

function saveErrorFigure(results, outputDir)
    if ~isfield(results, 'error') || isempty(results.error.time) || isempty(results.error.errors)
        return;
    end

    fig = [];
    try
        fig = figure('Name', 'Formation Tracking Error Over Time', 'Visible', 'off');
        ax = axes('Parent', fig);
        PlotFormationError(results.error, ax);
        saveErrorFigureFiles(fig, outputDir);
    catch err
        warning('CLCBS:SaveErrorFigureFailed', 'Failed to save error curve: %s', err.message);
    end

    if ~isempty(fig) && ishandle(fig)
        close(fig);
    end
end

function saveErrorFigureFiles(fig, outputDir)
    print(fig, fullfile(outputDir, 'formation_error_curve.png'), '-dpng', '-r600');
    try
        exportgraphics(fig, fullfile(outputDir, 'formation_error_curve.pdf'), 'ContentType', 'vector');
    catch
        print(fig, fullfile(outputDir, 'formation_error_curve.pdf'), '-dpdf', '-r300');
    end
    print(fig, fullfile(outputDir, 'formation_error_curve.eps'), '-depsc2', '-r300');
end

function saveFigure(results, fig, outputDir)
    createdFigure = false;
    try
        if isempty(fig) || ~ishandle(fig)
            fig = figure('Name', 'CL-CBS saved result', 'Visible', 'off');
            DrawMap(results.map, results.starts, results.goals, results.params);
            DrawTrajectory(results.solution, results.params);
            title('Car-Like Conflict-Based Search (MATLAB port)');
            createdFigure = true;
        end
        saveas(fig, fullfile(outputDir, 'trajectory.png'));
    catch err
        warning('CLCBS:SaveFigureFailed', 'Failed to save trajectory figure: %s', err.message);
        if createdFigure && exist('fig', 'var') && ishandle(fig)
            close(fig);
        end
        return;
    end

    if createdFigure
        close(fig);
    end
end

function writeMatrixCompat(filename, data)
    try
        writematrix(data, filename);
    catch
        csvwrite(filename, data);
    end
end

function params = unpackParams(v, params)
    names = {'r', 'deltat', 'penaltyTurning', 'penaltyReversing', ...
        'penaltyCOD', 'mapResolution', 'carWidth', 'LF', 'LB', ...
        'obsRadius', 'constraintWaitTime', 'maxHighLevelIterations', ...
        'maxLowLevelNodes', 'maxTime', 'goalTolerance', 'yawTolerance'};
    count = min(numel(names), numel(v));
    for i = 1:count
        params.(names{i}) = v(i);
    end
    params.xyResolution = params.r * params.deltat;
    params.yawResolution = params.deltat;
end

function v = packParams(params)
    v = [params.r, params.deltat, params.penaltyTurning, ...
        params.penaltyReversing, params.penaltyCOD, params.mapResolution, ...
        params.carWidth, params.LF, params.LB, params.obsRadius, ...
        params.constraintWaitTime, params.maxHighLevelIterations, ...
        params.maxLowLevelNodes, params.maxTime, params.goalTolerance, ...
        params.yawTolerance];
end

function value = pickField(s, fieldName, defaultValue)
    if isstruct(s) && isfield(s, fieldName)
        value = s.(fieldName);
    else
        value = defaultValue;
    end
end

function merged = mergeStruct(base, patch)
    merged = base;
    if isempty(patch)
        return;
    end
    names = fieldnames(patch);
    for i = 1:numel(names)
        f = names{i};
        if isstruct(patch.(f)) && isfield(merged, f) && isstruct(merged.(f))
            merged.(f) = mergeStruct(merged.(f), patch.(f));
        else
            merged.(f) = patch.(f);
        end
    end
end
