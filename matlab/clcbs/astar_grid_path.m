function [path, success] = astar_grid_path(startXY, goalXY, occ, cfg, smoothIters)
% astar_grid_path
% 8-neighbor A* in occupancy grid.
%
% Inputs:
%   startXY, goalXY : [x, y]
%   occ             : occupancy matrix (true means blocked)
%   smoothIters     : optional smoothing iteration count

    if nargin < 5
        smoothIters = 0;
    end

    [sx, sy] = world_to_grid(startXY, cfg);
    [gx, gy] = world_to_grid(goalXY, cfg);
    [nRows, nCols] = size(occ);

    if occ(sy, sx) || occ(gy, gx)
        path = [];
        success = false;
        return;
    end

    gScore = inf(nRows, nCols);
    fScore = inf(nRows, nCols);
    openMask = false(nRows, nCols);
    closedMask = false(nRows, nCols);
    parentX = zeros(nRows, nCols, 'int32');
    parentY = zeros(nRows, nCols, 'int32');

    gScore(sy, sx) = 0;
    fScore(sy, sx) = heuristic(sx, sy, gx, gy);
    openMask(sy, sx) = true;

    neighbor = [...
        -1, -1; 0, -1; 1, -1; ...
        -1,  0;         1,  0; ...
        -1,  1; 0,  1; 1,  1];
    moveCost = [sqrt(2), 1, sqrt(2), 1, 1, sqrt(2), 1, sqrt(2)] * cfg.gridResolution;

    reached = false;
    while any(openMask(:))
        openIdx = find(openMask);
        [~, localMin] = min(fScore(openIdx));
        current = openIdx(localMin);
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
        success = false;
        return;
    end

    pathGrid = [gx, gy];
    cx = gx;
    cy = gy;
    while ~(cx == sx && cy == sy)
        px = parentX(cy, cx);
        py = parentY(cy, cx);
        if px == 0 || py == 0
            path = [];
            success = false;
            return;
        end
        pathGrid = [px, py; pathGrid]; %#ok<AGROW>
        cx = px;
        cy = py;
    end

    path = grid_to_world_path(pathGrid, cfg);
    path(1, :) = startXY;
    path(end, :) = goalXY;

    if smoothIters > 0
        path = smooth_polyline(path, smoothIters);
        path(1, :) = startXY;
        path(end, :) = goalXY;
    end

    success = true;
end

function h = heuristic(x, y, gx, gy)
    h = hypot(double(gx - x), double(gy - y));
end
