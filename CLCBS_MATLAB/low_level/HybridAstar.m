function [plan, success, stats] = HybridAstar(mapData, start, goal, constraints, params, agentIdx)
%HYBRIDASTAR Spatiotemporal Hybrid A* low-level planner.
%   This MATLAB port keeps the CL-CBS motion primitives and constraint checks.
%   The OMPL Reeds-Shepp analytic expansion in the C++ code is replaced by a
%   lightweight goal-tolerance test plus an approximate Dubins-style connector.

    if nargin < 6
        agentIdx = 1; %#ok<NASGU>
    end

    stats = struct('expanded', 0, 'discovered', 0);
    plan = emptyPlan();
    success = false;

    startState = [start(1), start(2), normalizeHeading(start(3)), 0];
    goalState = [goal(1), goal(2), normalizeHeading(goal(3)), 0];

    if CollisionCheck(startState, mapData, params) || CollisionCheck(goalState, mapData, params)
        return;
    end

    startKey = stateKey(startState, params);
    startNode = makeNode(startState, 6, 0, DubinsHeuristic(startState, goalState, params), startKey);
    open = startNode;

    bestG = containers.Map('KeyType', 'char', 'ValueType', 'double');
    bestG(startKey) = 0;
    cameFrom = containers.Map('KeyType', 'char', 'ValueType', 'any');
    closed = containers.Map('KeyType', 'char', 'ValueType', 'logical');

    lastGoalConstraint = computeLastGoalConstraint(goalState, constraints, params);

    while ~isempty(open) && stats.expanded < params.maxLowLevelNodes
        currentIdx = popBestIndex(open);
        current = open(currentIdx);
        open(currentIdx) = [];

        if isKey(closed, current.key)
            continue;
        end
        closed(current.key) = true;
        stats.expanded = stats.expanded + 1;

        if isGoal(current.state, goalState, params) && current.state(4) > lastGoalConstraint
            [connector, connectorCost, connectorOk] = DubinsPlanner(current.state, goalState, params);
            if connectorOk && connectorValid(connector, mapData, constraints, params)
                [states, actions] = reconstructPath(current, startKey, cameFrom);
                if size(connector, 1) > 1
                    connector = stampConnectorTimes(connector, states(end, 4));
                    states = [states; connector(2:end, :)]; %#ok<AGROW>
                    actions = [actions; repmat([0, connectorCost / max(1, size(connector, 1) - 1)], size(connector, 1) - 1, 1)]; %#ok<AGROW>
                end
                plan.states = states;
                plan.actions = actions;
                plan.cost = current.g + connectorCost;
                plan.fmin = plan.cost;
                success = true;
                return;
            end
        end

        neighbors = getNeighbors(current.state, current.action, mapData, constraints, params);
        for i = 1:numel(neighbors)
            nb = neighbors(i);
            nbKey = stateKey(nb.state, params);
            if isKey(closed, nbKey)
                continue;
            end

            tentativeG = current.g + nb.cost;
            if ~isKey(bestG, nbKey) || tentativeG < bestG(nbKey)
                bestG(nbKey) = tentativeG;
                fScore = tentativeG + DubinsHeuristic(nb.state, goalState, params);
                open(end + 1) = makeNode(nb.state, nb.action, tentativeG, fScore, nbKey); %#ok<AGROW>
                cameFrom(nbKey) = struct('parentKey', current.key, ...
                    'parentState', current.state, 'action', nb.action, ...
                    'stepCost', nb.cost);
                stats.discovered = stats.discovered + 1;
            end
        end
    end
end

function plan = emptyPlan()
    plan = struct('states', zeros(0, 4), 'actions', zeros(0, 2), ...
        'cost', inf, 'fmin', inf);
end

function node = makeNode(state, action, g, f, key)
    node = struct('state', state, 'action', action, 'g', g, 'f', f, 'key', key);
end

function idx = popBestIndex(open)
    fValues = [open.f];
    minF = min(fValues);
    candidates = find(abs(fValues - minF) <= 1e-9);
    if numel(candidates) == 1
        idx = candidates;
        return;
    end
    gValues = [open(candidates).g];
    [~, localIdx] = max(gValues);
    idx = candidates(localIdx);
end

function neighbors = getNeighbors(state, previousAction, mapData, constraints, params)
    dx = [params.r * params.deltat, params.r * sin(params.deltat), ...
        params.r * sin(params.deltat), -params.r * params.deltat, ...
        -params.r * sin(params.deltat), -params.r * sin(params.deltat)];
    dy = [0, -params.r * (1 - cos(params.deltat)), ...
        params.r * (1 - cos(params.deltat)), 0, ...
        -params.r * (1 - cos(params.deltat)), params.r * (1 - cos(params.deltat))];
    dyaw = [0, params.deltat, -params.deltat, 0, -params.deltat, params.deltat];

    neighbors = repmat(struct('state', zeros(1, 4), 'action', 0, 'cost', 0), 0, 1);
    yaw = state(3);
    for action = 0:5
        k = action + 1;
        next = [ ...
            state(1) + dx(k) * cos(-yaw) - dy(k) * sin(-yaw), ...
            state(2) + dx(k) * sin(-yaw) + dy(k) * cos(-yaw), ...
            normalizeHeading(state(3) + dyaw(k)), ...
            state(4) + 1];

        stepCost = abs(dx(1));
        if mod(action, 3) ~= 0
            stepCost = stepCost * params.penaltyTurning;
        end
        if ((action < 3 && previousAction >= 3 && previousAction < 6) || ...
                (previousAction < 3 && action >= 3))
            stepCost = stepCost * params.penaltyCOD;
        end
        if action >= 3
            stepCost = stepCost * params.penaltyReversing;
        end

        if stateValid(next, mapData, constraints, params)
            neighbors(end + 1) = struct('state', next, 'action', action, 'cost', stepCost); %#ok<AGROW>
        end
    end

    waitState = [state(1), state(2), state(3), state(4) + 1];
    if stateValid(waitState, mapData, constraints, params)
        neighbors(end + 1) = struct('state', waitState, 'action', 6, 'cost', abs(dx(1))); %#ok<AGROW>
    end
end

function valid = stateValid(state, mapData, constraints, params)
    valid = state(4) <= params.maxTime && ...
        ~CollisionCheck(state, mapData, params) && ...
        CheckConstraint(state, constraints, params);
end

function tf = isGoal(state, goalState, params)
    distOk = hypot(goalState(1) - state(1), goalState(2) - state(2)) <= params.goalTolerance;
    yawOk = abs(wrapToPiLocal(goalState(3) - state(3))) <= params.yawTolerance;
    tf = distOk && yawOk;
end

function [states, actions] = reconstructPath(current, startKey, cameFrom)
    states = current.state;
    actions = zeros(0, 2);
    key = current.key;
    while ~strcmp(key, startKey)
        rec = cameFrom(key);
        states = [rec.parentState; states]; %#ok<AGROW>
        actions = [[rec.action, rec.stepCost]; actions]; %#ok<AGROW>
        key = rec.parentKey;
    end
end

function key = stateKey(state, params)
    ix = floor(state(1) / params.xyResolution);
    iy = floor(state(2) / params.xyResolution);
    iyaw = floor(normalizeHeading(state(3)) / params.yawResolution);
    it = round(state(4));
    key = sprintf('%d_%d_%d_%d', ix, iy, iyaw, it);
end

function lastTime = computeLastGoalConstraint(goalState, constraints, params)
    lastTime = -1;
    for i = 1:numel(constraints)
        cState = constraints(i).state;
        cState(4) = constraints(i).time;
        if CollisionCheck([goalState(1:3), constraints(i).time], cState, params)
            lastTime = max(lastTime, constraints(i).time);
        end
    end
end

function ok = connectorValid(path, mapData, constraints, params)
    ok = true;
    for i = 1:size(path, 1)
        if CollisionCheck(path(i, :), mapData, params) || ~CheckConstraint(path(i, :), constraints, params)
            ok = false;
            return;
        end
    end
end

function path = stampConnectorTimes(path, startTime)
    for i = 1:size(path, 1)
        path(i, 4) = startTime + i - 1;
    end
end

function yaw = normalizeHeading(yaw)
    yaw = mod(yaw, 2 * pi);
end

function angle = wrapToPiLocal(angle)
    angle = mod(angle + pi, 2 * pi) - pi;
end
