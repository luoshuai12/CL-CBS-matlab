function path = grid_to_world_path(pathGrid, cfg)
% grid_to_world_path
% Convert grid coordinates [gx, gy] to world [x, y].

    path = zeros(size(pathGrid, 1), 2);
    path(:, 1) = (double(pathGrid(:, 1)) - 1) * cfg.gridResolution;
    path(:, 2) = (double(pathGrid(:, 2)) - 1) * cfg.gridResolution;
end
