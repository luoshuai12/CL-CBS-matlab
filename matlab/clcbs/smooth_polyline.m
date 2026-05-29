function smoothPath = smooth_polyline(path, iterations)
% smooth_polyline
% Lightweight polyline smoothing while preserving endpoints.

    smoothPath = path;
    if size(path, 1) < 4 || iterations <= 0
        return;
    end

    for it = 1:iterations
        nextPath = smoothPath;
        nextPath(2:end-1, :) = ...
            0.2 * smoothPath(1:end-2, :) + ...
            0.6 * smoothPath(2:end-1, :) + ...
            0.2 * smoothPath(3:end, :);
        nextPath(1, :) = smoothPath(1, :);
        nextPath(end, :) = smoothPath(end, :);
        smoothPath = nextPath;
    end
end
