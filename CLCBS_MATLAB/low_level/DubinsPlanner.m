function [path, pathLength, success] = DubinsPlanner(startState, goalState, params)
%DUBINSPLANNER Lightweight curvature-aware connector used by HybridAstar.
%   This is an OMPL-free MATLAB approximation: it samples a smooth interpolation
%   between poses and accounts for heading change with the turning radius.

    if numel(startState) < 4
        startState(4) = 0;
    end
    if numel(goalState) < 4
        goalState(4) = startState(4) + 1;
    end

    xyDist = hypot(goalState(1) - startState(1), goalState(2) - startState(2));
    yawDelta = wrapToPiLocal(goalState(3) - startState(3));
    pathLength = xyDist + params.r * abs(yawDelta);

    n = max(2, ceil(pathLength / params.interpolationStep) + 1);
    alpha = linspace(0, 1, n).';

    path = zeros(n, 4);
    path(:, 1) = startState(1) + alpha * (goalState(1) - startState(1));
    path(:, 2) = startState(2) + alpha * (goalState(2) - startState(2));
    path(:, 3) = normalizeHeading(startState(3) + alpha * yawDelta);
    path(:, 4) = startState(4) + (0:n - 1).';

    success = all(isfinite(path(:)));
end

function yaw = normalizeHeading(yaw)
    yaw = mod(yaw, 2 * pi);
end

function angle = wrapToPiLocal(angle)
    angle = mod(angle + pi, 2 * pi) - pi;
end
