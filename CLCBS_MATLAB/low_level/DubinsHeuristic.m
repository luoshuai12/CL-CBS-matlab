function h = DubinsHeuristic(state, goalState, params)
%DUBINSHEURISTIC Curvature-aware admissible-style heuristic for Hybrid A*.
%   It combines Euclidean distance and the minimum turning-radius heading cost.

    if numel(state) < 3 || numel(goalState) < 3
        error('DubinsHeuristic expects states formatted as [x y yaw ...].');
    end

    xyCost = hypot(goalState(1) - state(1), goalState(2) - state(2));
    headingCost = params.r * abs(wrapToPiLocal(goalState(3) - state(3)));
    h = max(xyCost, headingCost);
end

function angle = wrapToPiLocal(angle)
    angle = mod(angle + pi, 2 * pi) - pi;
end
