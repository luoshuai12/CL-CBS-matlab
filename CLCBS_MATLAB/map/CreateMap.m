function [mapData, starts, goals, params] = CreateMap(config)
%CREATEMAP Build a small CL-CBS demonstration scenario.
%   [mapData, starts, goals, params] = CreateMap() returns a map struct,
%   start/goal states [x y yaw], and planner parameters.

    if nargin < 1
        config = struct();
    end

    params = defaultParams();
    params = mergeStruct(params, pickField(config, 'params', struct()));

    if isfield(config, 'mapSize')
        mapSize = config.mapSize;
    else
        mapSize = [60, 35];
    end

    if isfield(config, 'obstacles')
        obstacles = config.obstacles;
    else
        obstacles = [ ...
            23, 17, 2.8; ...
            30, 24, 2.4; ...
            34, 11, 2.5; ...
            42, 18, 3.0];
    end

    if isfield(config, 'starts')
        starts = config.starts;
    else
        starts = [ ...
             6,  8, 0; ...
             6, 17, 0; ...
             6, 26, 0];
    end

    if isfield(config, 'goals')
        goals = config.goals;
    else
        goals = [ ...
            54, 26, 0; ...
            54, 17, 0; ...
            54,  8, 0];
    end

    mapData = struct();
    mapData.size = mapSize;
    mapData.obstacles = obstacles;
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

    params.carWidth = 2.0;
    params.LF = 2.0;
    params.LB = 1.0;
    params.obsRadius = 0.8;
    params.constraintWaitTime = 2;

    params.maxHighLevelIterations = 80;
    params.maxLowLevelNodes = 45000;
    params.maxTime = 160;
    params.goalTolerance = 2.5;
    params.yawTolerance = pi / 3;
    params.agentSafetyScale = 1.0;
    params.interpolationStep = 0.8;
    params.preciseCollision = true;
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
