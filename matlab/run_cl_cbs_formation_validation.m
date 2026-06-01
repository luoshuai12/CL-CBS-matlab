function results = run_cl_cbs_formation_validation(userConfig)
%RUN_CL_CBS_FORMATION_VALIDATION
% MATLAB validation entry for a 7-robot formation experiment:
%   - Robot IDs 0,1,2 are leaders (MATLAB indices 1,2,3)
%   - Robot IDs 3,4,5,6 are followers
%   - Map size: 150m x 50m
%   - Robot footprint: 1m x 2m (inflated to a circular safety radius)
%   - Nine maps in total: 3 maps each for obstacle counts [10, 20, 30]
%
% Three methods are compared on every map:
%   1) Proposed method ("本文算法")
%   2) CL-CBS
%   3) CL-CBS-II (leaders by CL-CBS, followers by formation control law)
%
% Output metrics:
%   - Total formation completion time
%   - Formation error (mean of follower tracking errors to desired offsets)
%
% Usage:
%   results = run_cl_cbs_formation_validation();
%
% Optional override:
%   cfg.obstacleGroups = [8 16 24];
%   cfg.mapsPerGroup = 2;
%   results = run_cl_cbs_formation_validation(cfg);

    if nargin < 1
        userConfig = struct();
    end

    cfg = defaultConfig();
    cfg = mergeStruct(cfg, userConfig);

    if ~exist(cfg.outputDir, "dir")
        mkdir(cfg.outputDir);
    end

    rng(cfg.randomSeed);

    scenarioList = buildScenarioList(cfg);
    nCases = numel(scenarioList);
    algorithmKeys = {"proposed", "clcbs", "clcbs2"};
    algorithmNames = {"本文算法", "CL-CBS", "CL-CBS-II"};

    results = struct();
    results.config = cfg;
    results.algorithmKeys = algorithmKeys;
    results.algorithmNames = algorithmNames;
    results.cases = repmat(struct(), nCases, 1);

    fprintf("== CL-CBS 编队验证开始 ==\n");
    fprintf("地图大小: %.1f x %.1f m, 机器人数量: %d\n", cfg.mapSize(1), cfg.mapSize(2), cfg.numRobots);
    fprintf("障碍物组: %s, 每组地图数: %d\n\n", mat2str(cfg.obstacleGroups), cfg.mapsPerGroup);

    for caseIdx = 1:nCases
        caseConfig = scenarioList(caseIdx);
        fprintf("[Case %d/%d] obstacles=%d, seed=%d\n", ...
            caseIdx, nCases, caseConfig.obstacleCount, caseConfig.seed);

        map = generateValidMap(cfg, caseConfig.obstacleCount, caseConfig.seed);
        results.cases(caseIdx).caseId = caseIdx;
        results.cases(caseIdx).obstacleCount = caseConfig.obstacleCount;
        results.cases(caseIdx).seed = caseConfig.seed;
        results.cases(caseIdx).map = map;
        results.cases(caseIdx).runs = struct();

        for algIdx = 1:numel(algorithmKeys)
            key = algorithmKeys{algIdx};
            runResult = runSingleAlgorithm(cfg, map, key);
            results.cases(caseIdx).runs.(key) = runResult;
            fprintf("  - %-8s: total_time = %8.3f s, mean_error = %6.3f m\n", ...
                algorithmNames{algIdx}, runResult.totalTime, runResult.meanFormationError);
        end
        fprintf("\n");
    end

    summary = buildSummaryTables(results);
    results.summary = summary;

    if cfg.saveTables
        writetable(summary.totalTimeTable, fullfile(cfg.outputDir, "total_time_table.csv"));
        writetable(summary.errorTable, fullfile(cfg.outputDir, "formation_error_table.csv"));
    end

    save(fullfile(cfg.outputDir, "cl_cbs_formation_validation.mat"), "results");

    if cfg.plotFigures
        plotFormationLayout(cfg);
        plotSummaryCharts(results);
        if cfg.saveFigures
            saveas(gcf, fullfile(cfg.outputDir, "summary_metrics.png"));
        end
        for k = 1:nCases
            fig = plotCaseComparison(results, k);
            if cfg.saveFigures
                saveas(fig, fullfile(cfg.outputDir, sprintf("case_%02d_comparison.png", k)));
                close(fig);
            end
        end
    end

    fprintf("== CL-CBS 编队验证完成 ==\n\n");
    disp("总时间统计(秒):");
    disp(summary.totalTimeTable);
    disp("编队误差统计(米):");
    disp(summary.errorTable);
end

function cfg = defaultConfig()
    cfg = struct();

    cfg.mapSize = [150, 50];
    cfg.gridResolution = 1.0;
    cfg.numRobots = 7;
    cfg.robotSize = [1.0, 2.0];
    cfg.robotRadius = hypot(cfg.robotSize(1) / 2, cfg.robotSize(2) / 2);
    cfg.nominalSpeed = 2.0; % m/s

    cfg.leaderIdx = [1, 2, 3];      % Robot IDs 0,1,2
    cfg.followerIdx = [4, 5, 6, 7]; % Robot IDs 3,4,5,6

    cfg.startStates = [...
        30, 25, 0; ...
        24, 31, 0; ...
        24, 19, 0; ...
        18, 31, 0; ...
        18, 19, 0; ...
        12, 31, 0; ...
        12, 19, 0];

    cfg.goalStates = [...
        130, 25, 0; ...
        124, 31, 0; ...
        124, 19, 0; ...
        118, 31, 0; ...
        118, 19, 0; ...
        112, 31, 0; ...
        112, 19, 0];

    cfg.desiredOffsets = cfg.startStates(:, 1:2) - cfg.startStates(1, 1:2);

    cfg.obstacleGroups = [10, 20, 30];
    cfg.mapsPerGroup = 3;
    cfg.randomSeed = 20260529;
    cfg.maxMapAttempts = 60;
    cfg.maxObstacleSampleAttempts = 8000;
    cfg.obstacleRadiusRange = [1.2, 2.8];
    cfg.obstacleSpawnMargin = 4.0;
    cfg.startGoalClearance = 6.0;
    cfg.minObstacleGap = 0.8;

    cfg.interRobotDistance = max(cfg.robotSize) + 0.35;
    cfg.maxConflictResolveIter = 240;

    cfg.proposedLowClearance = 1.2;
    cfg.proposedHighClearance = 3.8;
    cfg.proposedSmoothIters = 2;
    cfg.clcbs2TrackingGain = 0.45;
    cfg.clcbs2RepulsionGain = 0.60;
    cfg.clcbs2RepulsionRange = 4.0;
    cfg.maxControlStep = 1.0;

    cfg.plotFigures = true;
    cfg.saveFigures = true;
    cfg.saveTables = true;
    cfg.outputDir = fullfile(fileparts(mfilename("fullpath")), "results");
end

function merged = mergeStruct(base, patch)
    merged = base;
    if isempty(patch)
        return;
    end

    patchFields = fieldnames(patch);
    for i = 1:numel(patchFields)
        f = patchFields{i};
        if isstruct(patch.(f)) && isfield(merged, f) && isstruct(merged.(f))
            merged.(f) = mergeStruct(merged.(f), patch.(f));
        else
            merged.(f) = patch.(f);
        end
    end
end

function scenarioList = buildScenarioList(cfg)
    total = numel(cfg.obstacleGroups) * cfg.mapsPerGroup;
    scenarioList = repmat(struct("obstacleCount", 0, "seed", 0), total, 1);
    cursor = 1;
    for groupIdx = 1:numel(cfg.obstacleGroups)
        count = cfg.obstacleGroups(groupIdx);
        for k = 1:cfg.mapsPerGroup
            scenarioList(cursor).obstacleCount = count;
            scenarioList(cursor).seed = cfg.randomSeed + groupIdx * 100 + k * 11;
            cursor = cursor + 1;
        end
    end
end

function map = generateValidMap(cfg, obstacleCount, seed)
    starts = cfg.startStates(:, 1:2);
    goals = cfg.goalStates(:, 1:2);
    anchors = [starts; goals];

    for attempt = 1:cfg.maxMapAttempts
        rng(seed + attempt - 1);
        obstacles = sampleObstacles(cfg, obstacleCount, anchors);
        if size(obstacles, 1) ~= obstacleCount
            continue;
        end

        occ = buildOccupancyGrid(cfg, obstacles);
        isFeasible = true;
        for rid = 1:cfg.numRobots
            [~, ok] = astarPlan(starts(rid, :), goals(rid, :), occ, cfg);
            if ~ok
                isFeasible = false;
                break;
            end
        end
        if isFeasible
            map = struct();
            map.obstacles = obstacles;
            map.occupancy = occ;
            map.seed = seed + attempt - 1;
            return;
        end
    end

    error("无法生成可通行地图，请放宽障碍物参数或提高 maxMapAttempts。");
end

function obstacles = sampleObstacles(cfg, obstacleCount, anchors)
    obstacles = zeros(obstacleCount, 3);
    accepted = 0;

    for trial = 1:cfg.maxObstacleSampleAttempts
        r = randRange(cfg.obstacleRadiusRange(1), cfg.obstacleRadiusRange(2));
        x = randRange(cfg.obstacleSpawnMargin, cfg.mapSize(1) - cfg.obstacleSpawnMargin);
        y = randRange(cfg.obstacleSpawnMargin, cfg.mapSize(2) - cfg.obstacleSpawnMargin);

        distToAnchors = vecnorm(anchors - [x, y], 2, 2);
        if any(distToAnchors < (cfg.startGoalClearance + r + cfg.robotRadius))
            continue;
        end

        if accepted > 0
            existing = obstacles(1:accepted, :);
            centerDist = vecnorm(existing(:, 1:2) - [x, y], 2, 2);
            minAllow = existing(:, 3) + r + cfg.minObstacleGap;
            if any(centerDist < minAllow)
                continue;
            end
        end

        accepted = accepted + 1;
        obstacles(accepted, :) = [x, y, r];
        if accepted == obstacleCount
            break;
        end
    end

    obstacles = obstacles(1:accepted, :);
end

function occ = buildOccupancyGrid(cfg, obstacles)
    xAxis = 0:cfg.gridResolution:cfg.mapSize(1);
    yAxis = 0:cfg.gridResolution:cfg.mapSize(2);
    [X, Y] = meshgrid(xAxis, yAxis);
    occ = false(size(X));

    inflated = cfg.robotRadius;
    for i = 1:size(obstacles, 1)
        obs = obstacles(i, :);
        occ = occ | hypot(X - obs(1), Y - obs(2)) <= (obs(3) + inflated);
    end

    for rid = 1:cfg.numRobots
        [sx, sy] = worldToGrid(cfg.startStates(rid, 1:2), cfg);
        [gx, gy] = worldToGrid(cfg.goalStates(rid, 1:2), cfg);
        occ(sy, sx) = false;
        occ(gy, gx) = false;
    end
end

function runResult = runSingleAlgorithm(cfg, map, method)
    starts = cfg.startStates(:, 1:2);
    goals = cfg.goalStates(:, 1:2);
    occ = map.occupancy;

    switch lower(method)
        case "clcbs"
            paths = cell(cfg.numRobots, 1);
            for rid = 1:cfg.numRobots
                [p, ok] = astarPlan(starts(rid, :), goals(rid, :), occ, cfg);
                if ~ok
                    p = straightPath(starts(rid, :), goals(rid, :), 80);
                    p = keepPathFree(p, map.obstacles, cfg);
                end
                paths{rid} = smoothPolyline(p, 1);
            end
            paths = resolveTemporalConflicts(paths, cfg);

        case "clcbs2"
            paths = generateLeaderFollowerPaths(cfg, map, false);

        case "proposed"
            paths = generateLeaderFollowerPaths(cfg, map, true);

        otherwise
            error("Unknown method: %s", method);
    end

    paths = ensureGoalConsistency(paths, goals, map.obstacles, cfg);
    synced = synchronizePaths(paths);

    pathLengths = zeros(cfg.numRobots, 1);
    for rid = 1:cfg.numRobots
        pathLengths(rid) = polylineLength(paths{rid});
    end

    runResult = struct();
    runResult.paths = paths;
    runResult.syncedPaths = synced;
    runResult.pathLengths = pathLengths;
    runResult.totalTime = max(pathLengths) / cfg.nominalSpeed;
    runResult.formationErrorSeries = formationErrorSeries(synced, cfg);
    runResult.meanFormationError = mean(runResult.formationErrorSeries);
end

function paths = generateLeaderFollowerPaths(cfg, map, useProposedBlend)
    occ = map.occupancy;
    starts = cfg.startStates(:, 1:2);
    goals = cfg.goalStates(:, 1:2);
    n = cfg.numRobots;
    paths = cell(n, 1);

    % Plan leaders by CL-CBS style (individual A* + temporal conflict handling)
    leaderPaths = cell(numel(cfg.leaderIdx), 1);
    for i = 1:numel(cfg.leaderIdx)
        rid = cfg.leaderIdx(i);
        [p, ok] = astarPlan(starts(rid, :), goals(rid, :), occ, cfg);
        if ~ok
            p = straightPath(starts(rid, :), goals(rid, :), 80);
            p = keepPathFree(p, map.obstacles, cfg);
        end
        leaderPaths{i} = smoothPolyline(p, 1);
    end
    leaderPaths = resolveTemporalConflicts(leaderPaths, cfg);
    for i = 1:numel(cfg.leaderIdx)
        paths{cfg.leaderIdx(i)} = leaderPaths{i};
    end

    lead0 = paths{1};

    for k = 1:numel(cfg.followerIdx)
        rid = cfg.followerIdx(k);
        desiredOffset = cfg.desiredOffsets(rid, :);

        translated = lead0 + desiredOffset;
        translated = clipPathToMap(translated, cfg);

        [ownPath, ok] = astarPlan(starts(rid, :), goals(rid, :), occ, cfg);
        if ~ok
            ownPath = straightPath(starts(rid, :), goals(rid, :), max(60, size(lead0, 1)));
        end

        if useProposedBlend
            sampleCount = max(size(translated, 1), size(ownPath, 1));
            translatedR = resamplePolyline(translated, sampleCount);
            ownR = resamplePolyline(ownPath, sampleCount);

            clearances = obstacleClearanceSeries(translatedR, map.obstacles, cfg);
            weights = (clearances - cfg.proposedLowClearance) ./ ...
                max(1e-6, cfg.proposedHighClearance - cfg.proposedLowClearance);
            weights = min(1, max(0, weights));
            blended = weights .* translatedR + (1 - weights) .* ownR;
            blended = keepPathFree(blended, map.obstacles, cfg);
            blended = smoothPolyline(blended, cfg.proposedSmoothIters);
            blended(1, :) = starts(rid, :);
            blended(end, :) = goals(rid, :);
            paths{rid} = blended;
        else
            follower = followerControlPath(starts(rid, :), translated, map.obstacles, cfg);
            follower = connectToGoalIfNeeded(follower, goals(rid, :), occ, map.obstacles, cfg);
            follower = smoothPolyline(follower, 1);
            follower(1, :) = starts(rid, :);
            follower(end, :) = goals(rid, :);
            paths{rid} = follower;
        end
    end

    paths = resolveTemporalConflicts(paths, cfg);
end

function path = followerControlPath(startPos, referencePath, obstacles, cfg)
    nSteps = size(referencePath, 1);
    path = zeros(nSteps, 2);
    path(1, :) = startPos;

    for t = 2:nSteps
        curr = path(t - 1, :);
        target = referencePath(min(t, nSteps), :);

        track = cfg.clcbs2TrackingGain * (target - curr);
        repel = obstacleRepulsion(curr, obstacles, cfg.clcbs2RepulsionGain, cfg.clcbs2RepulsionRange, cfg);
        step = track + repel;
        stepNorm = norm(step);
        if stepNorm > cfg.maxControlStep
            step = step / stepNorm * cfg.maxControlStep;
        end

        next = curr + step;
        next = pushPointOutOfObstacles(next, obstacles, cfg);
        path(t, :) = clampPointToMap(next, cfg);
    end
end

function outPath = connectToGoalIfNeeded(path, goal, occ, obstacles, cfg)
    outPath = path;
    if norm(outPath(end, :) - goal) < cfg.gridResolution
        outPath(end, :) = goal;
        return;
    end

    [tail, ok] = astarPlan(outPath(end, :), goal, occ, cfg);
    if ~ok
        tail = straightPath(outPath(end, :), goal, 30);
        tail = keepPathFree(tail, obstacles, cfg);
    end

    outPath = [outPath; tail(2:end, :)];
    outPath(end, :) = goal;
end

function [path, success] = astarPlan(startXY, goalXY, occ, cfg)
    [sx, sy] = worldToGrid(startXY, cfg);
    [gx, gy] = worldToGrid(goalXY, cfg);
    [nRows, nCols] = size(occ);

    if occ(sy, sx) || occ(gy, gx)
        success = false;
        path = [];
        return;
    end

    gScore = inf(nRows, nCols);
    fScore = inf(nRows, nCols);
    openMask = false(nRows, nCols);
    closedMask = false(nRows, nCols);
    parentX = zeros(nRows, nCols, "int32");
    parentY = zeros(nRows, nCols, "int32");

    gScore(sy, sx) = 0;
    fScore(sy, sx) = heuristic(sx, sy, gx, gy);
    openMask(sy, sx) = true;

    neighbor = [...
        -1, -1; 0, -1; 1, -1; ...
        -1,  0;         1,  0; ...
        -1,  1; 0,  1; 1,  1];
    moveCost = [sqrt(2), 1, sqrt(2), 1, 1, sqrt(2), 1, sqrt(2)] * cfg.gridResolution;

    success = false;
    reached = false;
    while any(openMask(:))
        openIdx = find(openMask);
        [~, localMinIdx] = min(fScore(openIdx));
        current = openIdx(localMinIdx);
        [cy, cx] = ind2sub(size(openMask), current);

        if cx == gx && cy == gy
            reached = true;
            break;
        end

        openMask(cy, cx) = false;
        closedMask(cy, cx) = true;

        for k = 1:size(neighbor, 1)
            nx = cx + neighbor(k, 1);
            ny = cy + neighbor(k, 2);
            if nx < 1 || ny < 1 || nx > nCols || ny > nRows
                continue;
            end
            if occ(ny, nx) || closedMask(ny, nx)
                continue;
            end

            tentativeG = gScore(cy, cx) + moveCost(k);
            if tentativeG < gScore(ny, nx)
                parentX(ny, nx) = cx;
                parentY(ny, nx) = cy;
                gScore(ny, nx) = tentativeG;
                fScore(ny, nx) = tentativeG + heuristic(nx, ny, gx, gy) * cfg.gridResolution;
                openMask(ny, nx) = true;
            end
        end
    end

    if ~reached
        path = [];
        return;
    end

    gxCur = gx;
    gyCur = gy;
    pathGrid = [gxCur, gyCur];
    while ~(gxCur == sx && gyCur == sy)
        px = parentX(gyCur, gxCur);
        py = parentY(gyCur, gxCur);
        if px == 0 || py == 0
            path = [];
            return;
        end
        pathGrid = [px, py; pathGrid]; %#ok<AGROW>
        gxCur = px;
        gyCur = py;
    end

    path = gridToWorldPath(pathGrid, cfg);
    path(1, :) = startXY;
    path(end, :) = goalXY;
    path = smoothPolyline(path, 1);
    success = true;
end

function h = heuristic(x, y, gx, gy)
    h = hypot(double(gx - x), double(gy - y));
end

function path = ensureGoalConsistency(paths, goals, obstacles, cfg)
    path = paths;
    for i = 1:numel(paths)
        p = path{i};
        if isempty(p)
            p = straightPath(cfg.startStates(i, 1:2), goals(i, :), 80);
        end
        if norm(p(end, :) - goals(i, :)) > 1e-6
            p = [p; goals(i, :)];
        else
            p(end, :) = goals(i, :);
        end
        p = keepPathFree(p, obstacles, cfg);
        path{i} = p;
    end
end

function paths = resolveTemporalConflicts(paths, cfg)
    for iter = 1:cfg.maxConflictResolveIter
        [hasConflict, robotId, insertIdx] = detectFirstConflict(paths, cfg.interRobotDistance);
        if ~hasConflict
            return;
        end
        p = paths{robotId};
        insertIdx = max(2, min(insertIdx, size(p, 1)));
        holdPoint = p(insertIdx - 1, :);
        p = [p(1:insertIdx - 1, :); holdPoint; p(insertIdx:end, :)];
        paths{robotId} = p;
    end
end

function [hasConflict, delayRobot, delayStep] = detectFirstConflict(paths, minDistance)
    hasConflict = false;
    delayRobot = 0;
    delayStep = 0;

    synced = synchronizePaths(paths);
    n = numel(synced);
    maxLen = size(synced{1}, 1);

    for t = 2:maxLen
        for i = 1:n - 1
            pi = synced{i}(t, :);
            piPrev = synced{i}(t - 1, :);
            for j = i + 1:n
                pj = synced{j}(t, :);
                pjPrev = synced{j}(t - 1, :);

                if norm(pi - pj) < minDistance
                    hasConflict = true;
                    delayRobot = max(i, j);
                    delayStep = t;
                    return;
                end

                edgeSwap = norm(piPrev - pj) < minDistance && norm(pjPrev - pi) < minDistance;
                if edgeSwap
                    hasConflict = true;
                    delayRobot = max(i, j);
                    delayStep = t;
                    return;
                end
            end
        end
    end
end

function synced = synchronizePaths(paths)
    n = numel(paths);
    lens = zeros(n, 1);
    for i = 1:n
        lens(i) = size(paths{i}, 1);
    end
    maxLen = max(lens);
    synced = cell(n, 1);
    for i = 1:n
        p = paths{i};
        if size(p, 1) < maxLen
            p = [p; repmat(p(end, :), maxLen - size(p, 1), 1)];
        end
        synced{i} = p;
    end
end

function errSeries = formationErrorSeries(syncedPaths, cfg)
    nT = size(syncedPaths{1}, 1);
    errSeries = zeros(nT, 1);
    for t = 1:nT
        leadPos = syncedPaths{1}(t, :);
        e = zeros(numel(cfg.followerIdx), 1);
        for k = 1:numel(cfg.followerIdx)
            rid = cfg.followerIdx(k);
            desired = leadPos + cfg.desiredOffsets(rid, :);
            e(k) = norm(syncedPaths{rid}(t, :) - desired);
        end
        errSeries(t) = mean(e);
    end
end

function summary = buildSummaryTables(results)
    n = numel(results.cases);
    obs = zeros(n, 1);
    tProp = zeros(n, 1);
    tClcbs = zeros(n, 1);
    tClcbs2 = zeros(n, 1);
    eProp = zeros(n, 1);
    eClcbs = zeros(n, 1);
    eClcbs2 = zeros(n, 1);

    for i = 1:n
        c = results.cases(i);
        obs(i) = c.obstacleCount;
        tProp(i) = c.runs.proposed.totalTime;
        tClcbs(i) = c.runs.clcbs.totalTime;
        tClcbs2(i) = c.runs.clcbs2.totalTime;
        eProp(i) = c.runs.proposed.meanFormationError;
        eClcbs(i) = c.runs.clcbs.meanFormationError;
        eClcbs2(i) = c.runs.clcbs2.meanFormationError;
    end

    caseId = (1:n)';
    summary = struct();
    summary.totalTimeTable = table(caseId, obs, tProp, tClcbs, tClcbs2, ...
        "VariableNames", {"CaseID", "ObstacleCount", "Proposed", "CL_CBS", "CL_CBS_II"});
    summary.errorTable = table(caseId, obs, eProp, eClcbs, eClcbs2, ...
        "VariableNames", {"CaseID", "ObstacleCount", "Proposed", "CL_CBS", "CL_CBS_II"});
end

function fig = plotCaseComparison(results, caseIdx)
    caseData = results.cases(caseIdx);
    cfg = results.config;

    fig = figure("Name", sprintf("Case %d", caseIdx), "Color", "w", "Position", [80 60 1200 760]);
    tiledlayout(2, 2, "Padding", "compact", "TileSpacing", "compact");

    nexttile;
    drawMapAndPaths(caseData.map, caseData.runs.proposed.paths, cfg);
    title(sprintf("Case %d - 本文算法", caseIdx));

    nexttile;
    drawMapAndPaths(caseData.map, caseData.runs.clcbs.paths, cfg);
    title(sprintf("Case %d - CL-CBS", caseIdx));

    nexttile;
    drawMapAndPaths(caseData.map, caseData.runs.clcbs2.paths, cfg);
    title(sprintf("Case %d - CL-CBS-II", caseIdx));

    nexttile;
    hold on;
    plot(caseData.runs.proposed.formationErrorSeries, "LineWidth", 1.8);
    plot(caseData.runs.clcbs.formationErrorSeries, "LineWidth", 1.8);
    plot(caseData.runs.clcbs2.formationErrorSeries, "LineWidth", 1.8);
    grid on;
    xlabel("时间步");
    ylabel("平均编队误差 (m)");
    legend({"本文算法", "CL-CBS", "CL-CBS-II"}, "Location", "northwest");
    title("编队误差曲线");
    hold off;
end

function plotFormationLayout(cfg)
    fig = figure("Name", "Formation Layout", "Color", "w", "Position", [120 120 900 350]);
    hold on;
    xlim([0 cfg.mapSize(1)]);
    ylim([0 cfg.mapSize(2)]);
    axis equal;
    grid on;
    title("图 5 编队队形");
    xlabel("x (m)");
    ylabel("y (m)");

    starts = cfg.startStates(:, 1:2);
    for i = 1:size(starts, 1)
        drawRobotRect(starts(i, :), cfg.robotSize, sprintf("%d", i - 1));
    end
    hold off;

    if cfg.saveFigures
        saveas(fig, fullfile(cfg.outputDir, "formation_layout.png"));
        close(fig);
    end
end

function plotSummaryCharts(results)
    cfg = results.config;
    obsGroups = cfg.obstacleGroups;
    nG = numel(obsGroups);

    meanTime = zeros(nG, 3);
    meanErr = zeros(nG, 3);
    for g = 1:nG
        mask = [results.cases.obstacleCount] == obsGroups(g);
        subset = results.cases(mask);
        meanTime(g, 1) = mean(arrayfun(@(x) x.runs.proposed.totalTime, subset));
        meanTime(g, 2) = mean(arrayfun(@(x) x.runs.clcbs.totalTime, subset));
        meanTime(g, 3) = mean(arrayfun(@(x) x.runs.clcbs2.totalTime, subset));
        meanErr(g, 1) = mean(arrayfun(@(x) x.runs.proposed.meanFormationError, subset));
        meanErr(g, 2) = mean(arrayfun(@(x) x.runs.clcbs.meanFormationError, subset));
        meanErr(g, 3) = mean(arrayfun(@(x) x.runs.clcbs2.meanFormationError, subset));
    end

    figure("Name", "Summary Metrics", "Color", "w", "Position", [100 100 1100 430]);
    tiledlayout(1, 2, "Padding", "compact", "TileSpacing", "compact");

    nexttile;
    bar(obsGroups, meanTime, "grouped");
    grid on;
    xlabel("障碍物数量");
    ylabel("平均总时间 (s)");
    legend({"本文算法", "CL-CBS", "CL-CBS-II"}, "Location", "northwest");
    title("总时间对比");

    nexttile;
    bar(obsGroups, meanErr, "grouped");
    grid on;
    xlabel("障碍物数量");
    ylabel("平均编队误差 (m)");
    legend({"本文算法", "CL-CBS", "CL-CBS-II"}, "Location", "northwest");
    title("编队误差对比");
end

function drawMapAndPaths(map, paths, cfg)
    hold on;
    xlim([0 cfg.mapSize(1)]);
    ylim([0 cfg.mapSize(2)]);
    axis equal;
    grid on;
    xlabel("x (m)");
    ylabel("y (m)");

    theta = linspace(0, 2 * pi, 64);
    for i = 1:size(map.obstacles, 1)
        obs = map.obstacles(i, :);
        x = obs(1) + obs(3) * cos(theta);
        y = obs(2) + obs(3) * sin(theta);
        fill(x, y, [0.2 0.2 0.2], "FaceAlpha", 0.25, "EdgeColor", [0.2 0.2 0.2]);
    end

    cmap = lines(numel(paths));
    for i = 1:numel(paths)
        p = paths{i};
        plot(p(:, 1), p(:, 2), "-", "Color", cmap(i, :), "LineWidth", 1.7);
        plot(p(1, 1), p(1, 2), "o", "Color", cmap(i, :), "MarkerFaceColor", cmap(i, :), "MarkerSize", 5);
        plot(p(end, 1), p(end, 2), "s", "Color", cmap(i, :), "MarkerFaceColor", "w", "MarkerSize", 5);
    end

    hold off;
end

function drawRobotRect(center, robotSize, labelText)
    x = center(1);
    y = center(2);
    w = robotSize(2); % 2m
    h = robotSize(1); % 1m
    rectangle("Position", [x - w/2, y - h/2, w, h], ...
        "EdgeColor", [0.25 0.25 0.25], "LineWidth", 2.2, "FaceColor", [0.72 0.72 0.72]);
    text(x, y, labelText, "HorizontalAlignment", "center", "VerticalAlignment", "middle", "FontSize", 14);
end

function clearances = obstacleClearanceSeries(path, obstacles, cfg)
    n = size(path, 1);
    clearances = inf(n, 1);
    for i = 1:n
        p = path(i, :);
        d = hypot(obstacles(:, 1) - p(1), obstacles(:, 2) - p(2)) - (obstacles(:, 3) + cfg.robotRadius);
        clearances(i) = min(d);
    end
end

function rep = obstacleRepulsion(point, obstacles, gain, influenceRange, cfg)
    rep = [0, 0];
    for i = 1:size(obstacles, 1)
        c = obstacles(i, 1:2);
        r = obstacles(i, 3) + cfg.robotRadius;
        vec = point - c;
        dist = norm(vec);
        boundaryDist = dist - r;

        if boundaryDist < influenceRange
            if dist < 1e-5
                dir = [1, 0];
            else
                dir = vec / dist;
            end
            scale = gain * max(0, (influenceRange - boundaryDist) / influenceRange);
            rep = rep + dir * scale;
        end
    end
end

function p = keepPathFree(path, obstacles, cfg)
    p = path;
    for i = 1:size(p, 1)
        p(i, :) = pushPointOutOfObstacles(p(i, :), obstacles, cfg);
        p(i, :) = clampPointToMap(p(i, :), cfg);
    end
end

function p = pushPointOutOfObstacles(point, obstacles, cfg)
    p = point;
    for iter = 1:5
        moved = false;
        for i = 1:size(obstacles, 1)
            c = obstacles(i, 1:2);
            safeR = obstacles(i, 3) + cfg.robotRadius + 0.2;
            v = p - c;
            d = norm(v);
            if d < safeR
                if d < 1e-6
                    v = [1, 0];
                    d = 1;
                end
                p = c + (v / d) * safeR;
                moved = true;
            end
        end
        if ~moved
            break;
        end
    end
end

function path = clipPathToMap(path, cfg)
    path(:, 1) = min(max(path(:, 1), 0), cfg.mapSize(1));
    path(:, 2) = min(max(path(:, 2), 0), cfg.mapSize(2));
end

function pt = clampPointToMap(point, cfg)
    pt = [...
        min(max(point(1), 0), cfg.mapSize(1)), ...
        min(max(point(2), 0), cfg.mapSize(2))];
end

function p = resamplePolyline(path, nSamples)
    if size(path, 1) <= 1
        p = repmat(path(1, :), nSamples, 1);
        return;
    end
    seg = vecnorm(diff(path, 1, 1), 2, 2);
    s = [0; cumsum(seg)];
    total = s(end);
    if total < 1e-6
        p = repmat(path(1, :), nSamples, 1);
        return;
    end
    query = linspace(0, total, nSamples)';
    p = [interp1(s, path(:, 1), query, "linear"), ...
         interp1(s, path(:, 2), query, "linear")];
end

function len = polylineLength(path)
    if size(path, 1) < 2
        len = 0;
        return;
    end
    len = sum(vecnorm(diff(path, 1, 1), 2, 2));
end

function p = smoothPolyline(path, iterations)
    p = path;
    if size(path, 1) < 4
        return;
    end
    for it = 1:iterations
        q = p;
        q(2:end-1, :) = 0.2 * p(1:end-2, :) + 0.6 * p(2:end-1, :) + 0.2 * p(3:end, :);
        q(1, :) = p(1, :);
        q(end, :) = p(end, :);
        p = q;
    end
end

function p = straightPath(startPt, goalPt, nPoints)
    t = linspace(0, 1, nPoints)';
    p = startPt + t .* (goalPt - startPt);
end

function [gx, gy] = worldToGrid(xy, cfg)
    gx = round(xy(1) / cfg.gridResolution) + 1;
    gy = round(xy(2) / cfg.gridResolution) + 1;
    gx = min(max(gx, 1), round(cfg.mapSize(1) / cfg.gridResolution) + 1);
    gy = min(max(gy, 1), round(cfg.mapSize(2) / cfg.gridResolution) + 1);
end

function path = gridToWorldPath(pathGrid, cfg)
    path = zeros(size(pathGrid, 1), 2);
    path(:, 1) = (double(pathGrid(:, 1)) - 1) * cfg.gridResolution;
    path(:, 2) = (double(pathGrid(:, 2)) - 1) * cfg.gridResolution;
end

function r = randRange(a, b)
    r = a + (b - a) * rand();
end

%% ================================================================
%  本文算法 (Proposed): Hybrid A* + 应力矩阵仿射编队 + 走廊优化
%% ================================================================

function paths = generateProposedAffineFormationPaths(cfg, map)
% 流程1+2+3 的集成: 本文算法完整实现
% 1) Hybrid A* 规划领航者路径 (阿克曼运动学约束)
% 2) 计算平衡应力矩阵 Omega, 解析计算跟随者路径
% 3) 安全走廊构建 + 二次规划轨迹精化

    occ     = map.occupancy;
    starts  = cfg.startStates(:, 1:2);
    goals   = cfg.goalStates(:, 1:2);
    n       = cfg.numRobots;
    lIdx    = cfg.leaderIdx;
    fIdx    = cfg.followerIdx;
    paths   = cell(n, 1);

    % ---- 步骤1: 领航者 Hybrid A* 路径规划 ----
    leaderRaw = cell(numel(lIdx), 1);
    for i = 1:numel(lIdx)
        rid = lIdx(i);
        startState = cfg.startStates(rid, :);
        goalXY     = goals(rid, :);

        [p3d, ok] = hybridAstarPlanLocal(startState, goalXY, occ, cfg);
        if ok
            leaderRaw{i} = p3d(:,1:2);
        else
            [p2d, ok2] = astarPlan(starts(rid,:), goalXY, occ, cfg);
            if ok2
                leaderRaw{i} = p2d;
            else
                leaderRaw{i} = straightPath(starts(rid,:), goalXY, 80);
                leaderRaw{i} = keepPathFree(leaderRaw{i}, map.obstacles, cfg);
            end
        end
        leaderRaw{i} = smoothPolyline(leaderRaw{i}, 2);
        leaderRaw{i}(1,:)   = starts(lIdx(i),:);
        leaderRaw{i}(end,:) = goalXY;
    end
    leaderRaw = resolveTemporalConflicts(leaderRaw, cfg);
    for i = 1:numel(lIdx)
        paths{lIdx(i)} = leaderRaw{i};
    end

    % ---- 步骤2: 计算应力矩阵, 解析求跟随者路径 ----
    Omega = computeStressMatrixLocal(cfg.nominalPositions);

    % 领航者1 路径重采样至 dt=1s
    leader1raw = paths{lIdx(1)};
    totalLen   = polylineLength(leader1raw);
    nSteps     = max(10, ceil(totalLen / cfg.nominalSpeed) + 2);
    leader1    = resamplePolyline(leader1raw, nSteps);
    thetaL1    = computeHeadingLocal(leader1);

    % 领航者2,3: 极坐标偏移 p_Lk(t) = p_L1(t) + R(θ(t))*d_1k
    P_star = cfg.nominalPositions;
    d_12   = P_star(lIdx(2),:) - P_star(lIdx(1),:);
    d_13   = P_star(lIdx(3),:) - P_star(lIdx(1),:);

    leader2 = zeros(nSteps,2);
    leader3 = zeros(nSteps,2);
    for t = 1:nSteps
        ct = cos(thetaL1(t)); st = sin(thetaL1(t));
        R  = [ct,-st; st,ct];
        leader2(t,:) = leader1(t,:) + (R*d_12')';
        leader3(t,:) = leader1(t,:) + (R*d_13')';
    end
    leader2 = clipPathToMap(leader2, cfg);
    leader3 = clipPathToMap(leader3, cfg);

    paths{lIdx(1)} = leader1;
    paths{lIdx(2)} = leader2;
    paths{lIdx(3)} = leader3;

    % 跟随者: p_f(t) = -Omega_ff^{-1} * Omega_fl * p_l(t)
    Omega_ff = Omega(fIdx, fIdx);
    Omega_fl = Omega(fIdx, lIdx);
    if rcond(Omega_ff) < 1e-10
        invOff = pinv(Omega_ff);
    else
        invOff = Omega_ff \ eye(numel(fIdx));
    end
    M_fl = -invOff * Omega_fl;

    for k = 1:numel(fIdx)
        paths{fIdx(k)} = zeros(nSteps,2);
    end
    for t = 1:nSteps
        p_l = [leader1(t,:); leader2(t,:); leader3(t,:)];
        p_f = M_fl * p_l;
        for k = 1:numel(fIdx)
            paths{fIdx(k)}(t,:) = p_f(k,:);
        end
    end
    for k = 1:numel(fIdx)
        paths{fIdx(k)} = clipPathToMap(paths{fIdx(k)}, cfg);
    end

    % ---- 步骤3: 安全走廊 + QP 轨迹优化 ----
    for rid = 1:n
        if isempty(paths{rid}) || size(paths{rid},1) < 2, continue; end
        corr = buildSafetyCorridorsLocal(paths{rid}, map.obstacles, cfg);
        paths{rid} = optimTrajQPLocal(paths{rid}, corr, starts(rid,:), goals(rid,:), cfg);
        paths{rid}(1,:)   = starts(rid,:);
        paths{rid}(end,:) = goals(rid,:);
    end

    paths = resolveTemporalConflicts(paths, cfg);
    paths = ensureGoalConsistency(paths, goals, map.obstacles, cfg);
end

% ------------------------------------------------------------------
%  Hybrid A* (阿克曼运动学) - 本地版本
% ------------------------------------------------------------------
function [path, success] = hybridAstarPlanLocal(startState, goalXY, occ, cfg)
% Hybrid A*: 状态 = [x,y,theta], 阿克曼模型节点展开

    v       = cfg.nominalSpeed;
    dt      = cfg.hybridDt;
    L       = cfg.wheelbase;
    nS      = cfg.numSteeringAngles;
    dMax    = cfg.maxSteeringAngle;
    hw      = cfg.hybridHWeight;
    maxN    = cfg.hybridMaxNodes;
    goalTol = cfg.hybridGoalTol;
    mapW    = cfg.mapSize(1);
    mapH    = cfg.mapSize(2);
    gRes    = cfg.gridResolution;

    xR  = cfg.hybridXRes;
    yR  = cfg.hybridYRes;
    thR = cfg.hybridThRes;
    nXG = floor(mapW/xR) + 2;
    nYG = floor(mapH/yR) + 2;
    nThG = floor(2*pi/thR) + 2;

    deltaSet = linspace(-dMax, dMax, nS);

    gBest  = inf(nXG, nYG, nThG);
    nSt    = zeros(maxN, 3);
    nGcost = zeros(maxN, 1);
    nFcost = zeros(maxN, 1);
    nPar   = zeros(maxN, 1, 'int32');
    nCnt   = 0;

    heapF   = zeros(maxN, 1);
    heapIdx = zeros(maxN, 1, 'int32');
    heapSz  = 0;

    nCnt = nCnt + 1;
    nSt(nCnt,:)  = startState;
    nGcost(nCnt) = 0;
    h0 = hw * norm(startState(1:2)-goalXY);
    nFcost(nCnt) = h0;
    nPar(nCnt)   = 0;
    heapSz = heapSz + 1;
    heapF(heapSz) = h0; heapIdx(heapSz) = nCnt;

    gi0 = stateToGridIdxLocal(startState, xR, yR, thR, nXG, nYG, nThG);
    gBest(gi0(1),gi0(2),gi0(3)) = 0;

    success  = false;
    goalNode = -1;

    while heapSz > 0 && nCnt < maxN
        [~, mi] = min(heapF(1:heapSz));
        currIdx = heapIdx(mi);
        heapF(mi)   = heapF(heapSz);
        heapIdx(mi) = heapIdx(heapSz);
        heapSz = heapSz - 1;

        curr  = nSt(currIdx,:);
        gCurr = nGcost(currIdx);

        if norm(curr(1:2)-goalXY) <= goalTol
            success = true; goalNode = currIdx; break;
        end

        for a = 1:nS
            delta  = deltaSet(a);
            th1    = curr(3);
            dth_dt = (v/L)*tan(delta);

            x_new  = curr(1) + dt*v*cos(th1 + 0.5*dt*dth_dt);
            y_new  = curr(2) + dt*v*sin(th1 + 0.5*dt*dth_dt);
            th_new = mod(th1 + dt*dth_dt + pi, 2*pi) - pi;

            if x_new<0||x_new>mapW||y_new<0||y_new>mapH, continue; end

            hit = false;
            for frac = [0.35, 0.7, 1.0]
                xi = curr(1)+frac*(x_new-curr(1));
                yi = curr(2)+frac*(y_new-curr(2));
                gxi = min(max(round(xi/gRes)+1,1), size(occ,2));
                gyi = min(max(round(yi/gRes)+1,1), size(occ,1));
                if occ(gyi,gxi), hit=true; break; end
            end
            if hit, continue; end

            newState = [x_new, y_new, th_new];
            gi  = stateToGridIdxLocal(newState, xR, yR, thR, nXG, nYG, nThG);
            gNew = gCurr + v*dt;

            if gNew >= gBest(gi(1),gi(2),gi(3)), continue; end
            gBest(gi(1),gi(2),gi(3)) = gNew;

            if nCnt < maxN
                nCnt = nCnt+1;
                nSt(nCnt,:)  = newState;
                nGcost(nCnt) = gNew;
                hNew         = hw*norm(newState(1:2)-goalXY);
                nFcost(nCnt) = gNew+hNew;
                nPar(nCnt)   = currIdx;
                heapSz = heapSz+1;
                heapF(heapSz) = gNew+hNew; heapIdx(heapSz) = nCnt;
            end
        end
    end

    if success
        buf  = zeros(nCnt,3);
        pLen = 0;
        idx  = goalNode;
        while idx > 0
            pLen = pLen+1;
            buf(pLen,:) = nSt(idx,:);
            idx = nPar(idx);
        end
        path = flip(buf(1:pLen,:),1);
        path(end,1:2) = goalXY;
    else
        path = [];
    end
end

function gi = stateToGridIdxLocal(s, xR, yR, thR, nXG, nYG, nThG)
    ix  = min(max(floor(s(1)/xR)+1, 1), nXG);
    iy  = min(max(floor(s(2)/yR)+1, 1), nYG);
    thn = mod(s(3), 2*pi);
    it  = min(max(floor(thn/thR)+1, 1), nThG);
    gi  = [ix, iy, it];
end

% ------------------------------------------------------------------
%  应力矩阵计算 - 本地版本
% ------------------------------------------------------------------
function Omega = computeStressMatrixLocal(P_star)
% Omega = I - Q*(Q'Q)^{-1}*Q', Q = [ones(N,1), P_star]
% 满足: Omega*1=0, Omega*P_star=0, rank(Omega)=N-3

    N = size(P_star, 1);
    Q = [ones(N,1), P_star];
    [Qq, ~] = qr(Q, 0);
    Omega   = eye(N) - Qq*Qq';
    Omega   = (Omega+Omega')/2;
    Omega(abs(Omega)<1e-12) = 0;
end

% ------------------------------------------------------------------
%  计算路径航向角 - 本地版本
% ------------------------------------------------------------------
function heading = computeHeadingLocal(path)
    N = size(path,1);
    heading = zeros(N,1);
    for i = 1:N-1
        dx = path(i+1,1)-path(i,1);
        dy = path(i+1,2)-path(i,2);
        if hypot(dx,dy) > 1e-6
            heading(i) = atan2(dy,dx);
        elseif i > 1
            heading(i) = heading(i-1);
        end
    end
    heading(N) = heading(N-1);
    kernel  = [0.1;0.2;0.4;0.2;0.1];
    heading = conv(heading, kernel, 'same');
end

% ------------------------------------------------------------------
%  安全走廊构建 - 本地版本
% ------------------------------------------------------------------
function corridors = buildSafetyCorridorsLocal(path, obstacles, cfg)
    N    = size(path,1);
    corr = zeros(N,4);
    step = cfg.gridResolution;
    hw   = cfg.robotRadius + 0.5;
    mapW = cfg.mapSize(1);
    mapH = cfg.mapSize(2);

    for k = 1:N
        cx = path(k,1); cy = path(k,2);
        xlo = max(0, cx-hw);  xhi = min(mapW, cx+hw);
        ylo = max(0, cy-hw);  yhi = min(mapH, cy+hw);

        % 向右扩展
        while xhi+step <= mapW
            if corridorEdgeClear(xhi+step, ylo, yhi, 'x', obstacles, cfg)
                xhi = xhi+step;
            else, break; end
        end
        % 向左扩展
        while xlo-step >= 0
            if corridorEdgeClear(xlo-step, ylo, yhi, 'x', obstacles, cfg)
                xlo = xlo-step;
            else, break; end
        end
        % 向上扩展
        while yhi+step <= mapH
            if corridorEdgeClear(yhi+step, xlo, xhi, 'y', obstacles, cfg)
                yhi = yhi+step;
            else, break; end
        end
        % 向下扩展
        while ylo-step >= 0
            if corridorEdgeClear(ylo-step, xlo, xhi, 'y', obstacles, cfg)
                ylo = ylo-step;
            else, break; end
        end
        corr(k,:) = [xlo, xhi, ylo, yhi];
    end
    corridors = corr;
end

function ok = corridorEdgeClear(edgeVal, lo, hi, dir, obstacles, cfg)
    ok = true;
    r  = cfg.robotRadius + 0.1;
    ns = 4;
    pts = linspace(lo, hi, ns);
    for i = 1:ns
        if strcmp(dir,'x'), pt = [edgeVal, pts(i)];
        else,               pt = [pts(i),  edgeVal]; end
        for oi = 1:size(obstacles,1)
            if norm(pt-obstacles(oi,1:2)) < obstacles(oi,3)+r
                ok = false; return;
            end
        end
    end
end

% ------------------------------------------------------------------
%  轨迹优化 (QP) - 本地版本
% ------------------------------------------------------------------
function optPath = optimTrajQPLocal(refPath, corridors, startXY, goalXY, cfg)
% 最小化 ||p-p_ref||^2 + lambda*||二阶差分||^2  受走廊约束

    T     = size(refPath,1);
    nVars = 2*T;
    lam   = 0.08;

    refFlat = reshape(refPath', nVars, 1);

    % 平滑正则化 (二阶差分)
    e   = ones(T,1);
    D2  = spdiags([e,-2*e,e], 0:2, max(1,T-2), T);
    Dreg = kron(D2, eye(2));
    H   = 2*(speye(nVars) + lam*(Dreg'*Dreg));
    fv  = -2*refFlat;

    lb = zeros(nVars,1); ub = zeros(nVars,1);
    for t = 1:T
        lb(2*t-1) = max(0,              corridors(t,1));
        ub(2*t-1) = min(cfg.mapSize(1), corridors(t,2));
        lb(2*t)   = max(0,              corridors(t,3));
        ub(2*t)   = min(cfg.mapSize(2), corridors(t,4));
    end

    Aeq = zeros(4,nVars); beq = zeros(4,1);
    Aeq(1,1)=1; beq(1)=startXY(1);
    Aeq(2,2)=1; beq(2)=startXY(2);
    Aeq(3,nVars-1)=1; beq(3)=goalXY(1);
    Aeq(4,nVars)=1;   beq(4)=goalXY(2);

    opts = optimoptions('quadprog','Display','none','MaxIterations',600);
    [xOpt,~,flag] = quadprog(H, fv, [], [], Aeq, beq, lb, ub, refFlat, opts);

    if flag > 0
        optPath = reshape(xOpt, 2, T)';
    else
        optPath = refPath;
        for t = 1:T
            optPath(t,1) = min(max(optPath(t,1), corridors(t,1)), corridors(t,2));
            optPath(t,2) = min(max(optPath(t,2), corridors(t,3)), corridors(t,4));
        end
    end
    optPath(1,:)   = startXY;
    optPath(end,:) = goalXY;
end
