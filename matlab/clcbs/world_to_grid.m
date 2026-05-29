function [gx, gy] = world_to_grid(xy, cfg)
% world_to_grid
% Convert [x, y] in meters to occupancy index [gx, gy].

    gx = round(xy(1) / cfg.gridResolution) + 1;
    gy = round(xy(2) / cfg.gridResolution) + 1;

    maxGX = round(cfg.mapSize(1) / cfg.gridResolution) + 1;
    maxGY = round(cfg.mapSize(2) / cfg.gridResolution) + 1;

    gx = min(max(gx, 1), maxGX);
    gy = min(max(gy, 1), maxGY);
end
