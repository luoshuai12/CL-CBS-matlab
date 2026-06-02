function [mapData, starts, goals, params] = CreateMap(config)
%CREATEMAP Build a CL-CBS demonstration scenario.
%   [mapData, starts, goals, params] = CreateMap() returns a map struct,
%   start/goal states [x y yaw], and planner parameters.

    if nargin < 1
        config = struct();
    end

    params = defaultParams();
    params = mergeStruct(params, pickField(config, 'params', struct()));

    mapSize = pickField(config, 'mapSize', [150, 50]);
    [defaultStarts, defaultGoals] = DefaultScenario();

    if isfield(config, 'starts')
        starts = config.starts;
    else
        starts = defaultStarts;
    end

    if isfield(config, 'goals')
        goals = config.goals;
    else
        goals = defaultGoals;
    end

    if isfield(config, 'obstacles')
        obstacles = config.obstacles;
    else
        obstacleCount = pickField(config, 'obstacleCount', 10);
        obstacleCount = normalizeObstacleCount(obstacleCount);
        seed = pickField(config, 'randomSeed', 20260602);
        obstacles = generateReachableObstacles(mapSize, starts, goals, obstacleCount, params, seed);
    end

    mapData = struct();
    mapData.size = mapSize;
    mapData.obstacles = obstacles;
    mapData.obstacleCount = size(obstacles, 1);
    mapData.dynamicObstacles = [];
end

function params = defaultParams()
    params = struct();
    params.r = 3.0;
    params.deltat = 0.706;
    params.penaltyTurning = 1.5;
    params.penaltyReversing = 2.0;
    params.penaltyCOD = 2.0;
    params.mapResolution = 2.0;
    params.xyResolution = params.r * params.deltat;
    params.yawResolution = params.deltat;

    params.carWidth = 1.0;
    params.LF = 1.0;
    params.LB = 1.0;
    params.obsRadius = 0.8;
    params.constraintWaitTime = 2;

    params.maxHighLevelIterations = 240;
    params.maxLowLevelNodes = 90000;
    params.maxTime = 260;
    params.goalTolerance = 2.5;
    params.yawTolerance = pi / 3;
    params.agentSafetyScale = 1.0;
    params.interpolationStep = 0.8;
    params.preciseCollision = true;
    params.randomObstacleRadiusRange = [1.0, 2.6];
    params.obstacleMargin = 4.0;
    params.startGoalClearance = 7.0;
    params.minObstacleGap = 1.0;
    params.maxMapAttempts = 80;
    params.maxObstacleSampleAttempts = 6000;
end

function count = normalizeObstacleCount(count)
    allowed = [10, 20, 30];
    [~, idx] = min(abs(allowed - count));
    count = allowed(idx);
end

function obstacles = generateReachableObstacles(mapSize, starts, goals, obstacleCount, params, seed)
    anchors = [starts(:, 1:2); goals(:, 1:2)];
    for attempt = 1:params.maxMapAttempts
        rng(seed + attempt - 1);
        obstacles = sampleObstacles(mapSize, obstacleCount, anchors, params);
        if size(obstacles, 1) ~= obstacleCount
            continue;
        end
        if allAgentsReachable(mapSize, obstacles, starts, goals, params)
            return;
        end
    end

    error('CreateMap:NoReachableMap', ...
        '无法生成可达地图，请降低障碍物数量或调整起终点/障碍物参数。');
end

function obstacles = sampleObstacles(mapSize, obstacleCount, anchors, params)
    obstacles = zeros(obstacleCount, 3);
    accepted = 0;
    radiusRange = params.randomObstacleRadiusRange;
    margin = params.obstacleMargin;

    for trial = 1:params.maxObstacleSampleAttempts
        radius = radiusRange(1) + rand() * (radiusRange(2) - radiusRange(1));
        x = margin + radius + rand() * (mapSize(1) - 2 * (margin + radius));
        y = margin + radius + rand() * (mapSize(2) - 2 * (margin + radius));

        distToAnchors = sqrt(sum((anchors - [x, y]).^2, 2));
        if any(distToAnchors < params.startGoalClearance + radius)
            continue;
        end

        if accepted > 0
            existing = obstacles(1:accepted, :);
            centerDist = sqrt(sum((existing(:, 1:2) - [x, y]).^2, 2));
            minAllowed = existing(:, 3) + radius + params.minObstacleGap;
            if any(centerDist < minAllowed)
                continue;
            end
        end

        accepted = accepted + 1;
        obstacles(accepted, :) = [x, y, radius];
        if accepted == obstacleCount
            break;
        end
    end

    obstacles = obstacles(1:accepted, :);
end

function reachable = allAgentsReachable(mapSize, obstacles, starts, goals, params)
    reachable = true;
    for i = 1:size(starts, 1)
        if ~gridReachable(mapSize, obstacles, starts(i, 1:2), goals(i, 1:2), params)
            reachable = false;
            return;
        end
    end
end

function reachable = gridReachable(mapSize, obstacles, startPt, goalPt, params)
    resolution = 1.0;
    nx = floor(mapSize(1) / resolution) + 1;
    ny = floor(mapSize(2) / resolution) + 1;
    occupied = false(nx, ny);
    inflation = hypot(params.carWidth / 2, (params.LF + params.LB) / 2);

    for ix = 1:nx
        x = (ix - 1) * resolution;
        for iy = 1:ny
            y = (iy - 1) * resolution;
            for k = 1:size(obstacles, 1)
                if hypot(x - obstacles(k, 1), y - obstacles(k, 2)) <= obstacles(k, 3) + inflation
                    occupied(ix, iy) = true;
                    break;
                end
            end
        end
    end

    startIdx = pointToGrid(startPt, resolution, nx, ny);
    goalIdx = pointToGrid(goalPt, resolution, nx, ny);
    if occupied(startIdx(1), startIdx(2)) || occupied(goalIdx(1), goalIdx(2))
        reachable = false;
        return;
    end

    visited = false(nx, ny);
    queue = zeros(nx * ny, 2);
    head = 1;
    tail = 1;
    queue(tail, :) = startIdx;
    visited(startIdx(1), startIdx(2)) = true;
    motions = [1 0; -1 0; 0 1; 0 -1; 1 1; 1 -1; -1 1; -1 -1];

    reachable = false;
    while head <= tail
        node = queue(head, :);
        head = head + 1;
        if isequal(node, goalIdx)
            reachable = true;
            return;
        end
        for m = 1:size(motions, 1)
            next = node + motions(m, :);
            if next(1) < 1 || next(1) > nx || next(2) < 1 || next(2) > ny
                continue;
            end
            if occupied(next(1), next(2)) || visited(next(1), next(2))
                continue;
            end
            tail = tail + 1;
            queue(tail, :) = next;
            visited(next(1), next(2)) = true;
        end
    end
end

function idx = pointToGrid(point, resolution, nx, ny)
    idx = floor(point / resolution) + 1;
    idx(1) = min(max(idx(1), 1), nx);
    idx(2) = min(max(idx(2), 1), ny);
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
        merged.(names{i}) = patch.(names{i});
    end
end
