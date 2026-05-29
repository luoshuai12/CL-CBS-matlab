function metric = compute_clcbs_metrics(cfg, runData)
% compute_clcbs_metrics
% Compute total completion time and formation error for CL-CBS result.

    paths = runData.paths;
    synced = runData.syncedPaths;

    pathLens = zeros(cfg.numRobots, 1);
    for i = 1:cfg.numRobots
        pathLens(i) = polyline_length(paths{i});
    end

    totalTime = max(pathLens) / cfg.nominalSpeed;
    errSeries = formation_error_series(cfg, synced);

    metric = struct();
    metric.pathLengths = pathLens;
    metric.totalTime = totalTime;
    metric.formationErrorSeries = errSeries;
    metric.meanFormationError = mean(errSeries);
end

function errSeries = formation_error_series(cfg, syncedPaths)
    nStep = size(syncedPaths{1}, 1);
    errSeries = zeros(nStep, 1);

    for t = 1:nStep
        leaderPos = syncedPaths{1}(t, :);
        followerErr = zeros(numel(cfg.followerIdx), 1);

        for k = 1:numel(cfg.followerIdx)
            rid = cfg.followerIdx(k);
            desired = leaderPos + cfg.desiredOffsets(rid, :);
            followerErr(k) = norm(syncedPaths{rid}(t, :) - desired);
        end

        errSeries(t) = mean(followerErr);
    end
end
