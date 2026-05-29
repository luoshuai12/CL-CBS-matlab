function mapSet = generate_map_set(cfg, scenarios)
% generate_map_set
% Generate random map set and save each case into MapData folder.

    nCases = numel(scenarios);
    mapSet = repmat(struct('caseId', 0, 'obstacleCount', 0, 'seed', 0, ...
        'obstacles', [], 'occupancy', []), nCases, 1);

    for i = 1:nCases
        mapData = generate_one_map(cfg, scenarios(i));
        mapSet(i) = mapData;

        mapFile = fullfile(cfg.mapDataDir, sprintf('case_%02d_map.mat', mapData.caseId));
        save(mapFile, 'mapData');

        if cfg.saveMapPreview
            save_map_preview(cfg, mapData);
        end
    end
end

function mapData = generate_one_map(cfg, scenario)
    starts = cfg.startStates(:, 1:2);
    goals = cfg.goalStates(:, 1:2);
    anchors = [starts; goals];

    mapFound = false;
    chosenSeed = scenario.seed;
    obstacles = [];
    occ = [];

    for attempt = 1:cfg.maxMapAttempts
        currentSeed = scenario.seed + attempt - 1;
        rng(currentSeed);

        trialObstacles = sample_obstacles(cfg, scenario.obstacleCount, anchors);
        if size(trialObstacles, 1) ~= scenario.obstacleCount
            continue;
        end

        trialOcc = build_occupancy_grid(cfg, trialObstacles);
        if is_map_feasible(cfg, trialOcc)
            mapFound = true;
            chosenSeed = currentSeed;
            obstacles = trialObstacles;
            occ = trialOcc;
            break;
        end
    end

    if ~mapFound
        error('Case %d 地图生成失败：请放宽障碍参数或增大 maxMapAttempts。', scenario.caseId);
    end

    mapData = struct();
    mapData.caseId = scenario.caseId;
    mapData.obstacleCount = scenario.obstacleCount;
    mapData.seed = chosenSeed;
    mapData.obstacles = obstacles;
    mapData.occupancy = occ;
end

function obstacles = sample_obstacles(cfg, obstacleCount, anchors)
    obstacles = zeros(obstacleCount, 3);
    accepted = 0;

    for trial = 1:cfg.maxObstacleSampleAttempts
        radius = rand_range(cfg.obstacleRadiusRange(1), cfg.obstacleRadiusRange(2));
        x = rand_range(cfg.obstacleSpawnMargin, cfg.mapSize(1) - cfg.obstacleSpawnMargin);
        y = rand_range(cfg.obstacleSpawnMargin, cfg.mapSize(2) - cfg.obstacleSpawnMargin);

        d = vecnorm(anchors - [x, y], 2, 2);
        if any(d < (cfg.startGoalClearance + radius + cfg.robotRadius))
            continue;
        end

        if accepted > 0
            existing = obstacles(1:accepted, :);
            centerDist = vecnorm(existing(:, 1:2) - [x, y], 2, 2);
            minDist = existing(:, 3) + radius + cfg.minObstacleGap;
            if any(centerDist < minDist)
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

function occ = build_occupancy_grid(cfg, obstacles)
    xAxis = 0:cfg.gridResolution:cfg.mapSize(1);
    yAxis = 0:cfg.gridResolution:cfg.mapSize(2);
    [X, Y] = meshgrid(xAxis, yAxis);

    occ = false(size(X));
    inflatedRobot = cfg.robotRadius;

    for i = 1:size(obstacles, 1)
        obs = obstacles(i, :);
        occ = occ | (hypot(X - obs(1), Y - obs(2)) <= (obs(3) + inflatedRobot));
    end

    for rid = 1:cfg.numRobots
        [sx, sy] = world_to_grid(cfg.startStates(rid, 1:2), cfg);
        [gx, gy] = world_to_grid(cfg.goalStates(rid, 1:2), cfg);
        occ(sy, sx) = false;
        occ(gy, gx) = false;
    end
end

function ok = is_map_feasible(cfg, occ)
    ok = true;
    starts = cfg.startStates(:, 1:2);
    goals = cfg.goalStates(:, 1:2);

    for rid = 1:cfg.numRobots
        [~, success] = astar_grid_path(starts(rid, :), goals(rid, :), occ, cfg, 0);
        if ~success
            ok = false;
            return;
        end
    end
end

function save_map_preview(cfg, mapData)
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [120 120 900 320]);
    hold on;
    axis equal;
    xlim([0, cfg.mapSize(1)]);
    ylim([0, cfg.mapSize(2)]);
    grid on;
    xlabel('x (m)');
    ylabel('y (m)');
    title(sprintf('Case %02d - Obstacles: %d', mapData.caseId, mapData.obstacleCount));

    theta = linspace(0, 2 * pi, 60);
    for i = 1:size(mapData.obstacles, 1)
        obs = mapData.obstacles(i, :);
        fill(obs(1) + obs(3) * cos(theta), obs(2) + obs(3) * sin(theta), ...
            [0.35, 0.35, 0.35], 'FaceAlpha', 0.25, 'EdgeColor', [0.2, 0.2, 0.2]);
    end

    starts = cfg.startStates(:, 1:2);
    goals = cfg.goalStates(:, 1:2);
    plot(starts(:, 1), starts(:, 2), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 5);
    plot(goals(:, 1), goals(:, 2), 'rs', 'MarkerFaceColor', 'w', 'MarkerSize', 5);

    for rid = 1:cfg.numRobots
        text(starts(rid, 1) + 0.8, starts(rid, 2) + 0.5, sprintf('S%d', rid - 1), 'Color', 'b');
        text(goals(rid, 1) + 0.8, goals(rid, 2) + 0.5, sprintf('G%d', rid - 1), 'Color', 'r');
    end

    hold off;
    outPng = fullfile(cfg.mapImageDir, sprintf('case_%02d_map.png', mapData.caseId));
    saveas(fig, outPng);
    close(fig);
end

function value = rand_range(a, b)
    value = a + (b - a) * rand();
end
