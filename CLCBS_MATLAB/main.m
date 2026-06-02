function results = main()
%MAIN Entry point for the MATLAB CL-CBS port.
%   Run from this directory with:
%       results = main();

    clc;

    rootDir = fileparts(mfilename('fullpath'));
    addpath(fullfile(rootDir, 'map'));
    addpath(fullfile(rootDir, 'high_level'));
    addpath(fullfile(rootDir, 'low_level'));
    addpath(fullfile(rootDir, 'utils'));

    scenarioFile = fullfile(rootDir, 'data', 'scenario.mat');
    if exist(scenarioFile, 'file')
        [mapData, starts, goals, params] = loadScenario(scenarioFile);
    else
        [mapData, starts, goals, params] = CreateMap();
        saveScenario(scenarioFile, mapData, starts, goals, params);
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

    if success
        fprintf('Success. cost=%.3f, makespan=%d, high-level expanded=%d, runtime=%.3fs\n', ...
            stats.cost, stats.makespan, stats.highLevelExpanded, stats.runtime);
    else
        fprintf('Failed to find collision-free paths. high-level expanded=%d, runtime=%.3fs\n', ...
            stats.highLevelExpanded, stats.runtime);
    end

    figure('Name', 'CL-CBS MATLAB');
    DrawMap(mapData, starts, goals, params);
    if success
        DrawTrajectory(solution, params);
    end
    title('Car-Like Conflict-Based Search (MATLAB port)');
end

function [mapData, starts, goals, params] = loadScenario(filename)
    raw = load(filename);
    if isfield(raw, 'mapData')
        mapData = raw.mapData;
        starts = raw.starts;
        goals = raw.goals;
        params = raw.params;
        return;
    end

    mapData = struct();
    mapData.size = raw.mapSize(:).';
    mapData.obstacles = raw.obstacles;
    mapData.dynamicObstacles = [];
    starts = raw.starts;
    goals = raw.goals;

    [~, ~, ~, params] = CreateMap();
    if isfield(raw, 'paramsVector')
        params = unpackParams(raw.paramsVector, params);
    end
end

function saveScenario(filename, mapData, starts, goals, params)
    outDir = fileparts(filename);
    if ~exist(outDir, 'dir')
        mkdir(outDir);
    end
    mapSize = mapData.size; %#ok<NASGU>
    obstacles = mapData.obstacles; %#ok<NASGU>
    paramsVector = packParams(params); %#ok<NASGU>
    save(filename, 'mapSize', 'obstacles', 'starts', 'goals', 'paramsVector');
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
