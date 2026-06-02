function collides = CollisionCheck(stateA, target, params)
%COLLISIONCHECK Collision tests for CL-CBS states, maps, and obstacles.
%   state format: [x y yaw t]. Obstacle format: [x y radius].

    if nargin < 3
        error('CollisionCheck requires params.');
    end

    stateA = ensureState(stateA);

    if isstruct(target)
        collides = mapCollision(stateA, target, params);
        return;
    end

    if isnumeric(target) && isvector(target) && numel(target) >= 3
        stateB = ensureState(target);
        collides = agentCollision(stateA, stateB, params);
        return;
    end

    if isnumeric(target)
        collides = obstacleSetCollision(stateA, target, params);
        return;
    end

    error('Unsupported collision target.');
end

function state = ensureState(state)
    state = state(:).';
    if numel(state) < 4
        state(4) = 0;
    end
end

function collides = mapCollision(state, mapData, params)
    collides = true;
    if state(1) < 0 || state(1) > mapData.size(1) || state(2) < 0 || state(2) > mapData.size(2)
        return;
    end

    if isfield(mapData, 'obstacles') && obstacleSetCollision(state, mapData.obstacles, params)
        return;
    end

    if isfield(mapData, 'dynamicObstacles') && ~isempty(mapData.dynamicObstacles)
        dyn = mapData.dynamicObstacles;
        active = false(size(dyn, 1), 1);
        if size(dyn, 2) >= 4
            active = dyn(:, 4) == state(4);
            active = active | (dyn(:, 4) < 0 & state(4) >= abs(dyn(:, 4)));
        end
        for i = find(active).'
            if agentCollision(state, dyn(i, :), params)
                return;
            end
        end
    end

    collides = false;
end

function collides = obstacleSetCollision(state, obstacles, params)
    collides = false;
    if isempty(obstacles)
        return;
    end
    for i = 1:size(obstacles, 1)
        obstacle = obstacles(i, :);
        radius = params.obsRadius;
        if numel(obstacle) >= 3
            radius = obstacle(3);
        end
        if vehicleCircleCollision(state, obstacle(1:2), radius, params)
            collides = true;
            return;
        end
    end
end

function collides = vehicleCircleCollision(state, center, radius, params)
    dx = center(1) - state(1);
    dy = center(2) - state(2);
    yaw = state(3);

    localX = dx * cos(yaw) + dy * sin(yaw);
    localY = -dx * sin(yaw) + dy * cos(yaw);

    minX = -params.LB;
    maxX = params.LF;
    halfW = params.carWidth / 2;
    closestX = min(max(localX, minX), maxX);
    closestY = min(max(localY, -halfW), halfW);

    collides = hypot(localX - closestX, localY - closestY) <= radius;
end

function collides = agentCollision(stateA, stateB, params)
    safetyScale = params.agentSafetyScale;
    quickRadius = safetyScale * hypot(params.LF + params.LB, params.carWidth);
    if hypot(stateA(1) - stateB(1), stateA(2) - stateB(2)) > 2 * quickRadius
        collides = false;
        return;
    end

    if ~isfield(params, 'preciseCollision') || ~params.preciseCollision
        threshold = safetyScale * sqrt((2 * params.LF)^2 + params.carWidth^2);
        collides = hypot(stateA(1) - stateB(1), stateA(2) - stateB(2)) < threshold;
        return;
    end

    rectA = vehicleCorners(stateA, params, safetyScale);
    rectB = vehicleCorners(stateB, params, safetyScale);
    collides = rectanglesOverlap(rectA, rectB);
end

function corners = vehicleCorners(state, params, safetyScale)
    halfW = params.carWidth * safetyScale / 2;
    lf = params.LF * safetyScale;
    lb = params.LB * safetyScale;
    local = [ ...
        lf,  halfW; ...
        lf, -halfW; ...
       -lb, -halfW; ...
       -lb,  halfW];
    yaw = state(3);
    rot = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
    corners = local * rot.' + state(1:2);
end

function overlap = rectanglesOverlap(polyA, polyB)
    axesToTest = zeros(4, 2);
    axesToTest(1, :) = edgeNormal(polyA(2, :) - polyA(1, :));
    axesToTest(2, :) = edgeNormal(polyA(3, :) - polyA(2, :));
    axesToTest(3, :) = edgeNormal(polyB(2, :) - polyB(1, :));
    axesToTest(4, :) = edgeNormal(polyB(3, :) - polyB(2, :));

    overlap = true;
    for i = 1:size(axesToTest, 1)
        axis = axesToTest(i, :);
        projA = polyA * axis.';
        projB = polyB * axis.';
        if max(projA) < min(projB) || max(projB) < min(projA)
            overlap = false;
            return;
        end
    end
end

function n = edgeNormal(edge)
    n = [-edge(2), edge(1)];
    len = hypot(n(1), n(2));
    if len > 0
        n = n / len;
    end
end
