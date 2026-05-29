function runData = run_clcbs_case(cfg, mapData)
% run_clcbs_case
% Run CL-CBS-style planning on one map:
%   1) single-agent A* paths
%   2) temporal conflict insertion (wait strategy)

    starts = cfg.startStates(:, 1:2);
    goals = cfg.goalStates(:, 1:2);
    occ = mapData.occupancy;

    paths = cell(cfg.numRobots, 1);

    for rid = 1:cfg.numRobots
        [p, ok] = astar_grid_path(starts(rid, :), goals(rid, :), occ, cfg, 1);
        if ~ok
            p = straight_path(starts(rid, :), goals(rid, :), 90);
        end
        p(1, :) = starts(rid, :);
        p(end, :) = goals(rid, :);
        paths{rid} = p;
    end

    paths = resolve_temporal_conflicts(paths, cfg.interRobotDistance, cfg.maxConflictResolveIter);
    syncedPaths = synchronize_paths(paths);

    runData = struct();
    runData.paths = paths;
    runData.syncedPaths = syncedPaths;
end
