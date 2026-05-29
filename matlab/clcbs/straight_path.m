function path = straight_path(startPt, goalPt, nPoints)
% straight_path
% Generate line interpolation between start and goal.

    if nargin < 3
        nPoints = 80;
    end
    t = linspace(0, 1, nPoints)';
    path = startPt + t .* (goalPt - startPt);
end
