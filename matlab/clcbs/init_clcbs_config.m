function cfg = init_clcbs_config()
% init_clcbs_config
% Default experiment configuration for CL-CBS validation.

    thisDir = fileparts(mfilename('fullpath'));

    cfg = struct();

    cfg.mapSize = [150, 50];
    cfg.gridResolution = 1.0;

    cfg.numRobots = 7;
    cfg.robotSize = [1.0, 2.0]; % [width, length]
    cfg.robotRadius = hypot(cfg.robotSize(1) / 2, cfg.robotSize(2) / 2);
    cfg.nominalSpeed = 2.0; % m/s

    % Robot IDs 0..6 correspond to MATLAB indices 1..7.
    cfg.leaderIdx = [1, 2, 3];
    cfg.followerIdx = [4, 5, 6, 7];

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

    cfg.maxMapAttempts = 80;
    cfg.maxObstacleSampleAttempts = 9000;
    cfg.obstacleRadiusRange = [1.2, 2.8];
    cfg.obstacleSpawnMargin = 4.0;
    cfg.startGoalClearance = 6.0;
    cfg.minObstacleGap = 0.8;

    cfg.interRobotDistance = max(cfg.robotSize) + 0.35;
    cfg.maxConflictResolveIter = 260;

    cfg.saveMapPreview = true;
    cfg.saveCaseFigure = true;
    cfg.saveSummaryFigure = true;
    cfg.saveTables = true;

    cfg.mapDataDir = fullfile(thisDir, 'MapData');
    cfg.mapImageDir = fullfile(thisDir, 'ShapeImage');
    cfg.resultDir = fullfile(thisDir, 'Results');
end
