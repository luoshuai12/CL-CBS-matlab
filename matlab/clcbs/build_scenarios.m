function scenarios = build_scenarios(cfg)
% build_scenarios
% Build scenario list for all obstacle groups.

    totalCases = numel(cfg.obstacleGroups) * cfg.mapsPerGroup;
    scenarios = repmat(struct('caseId', 0, 'obstacleCount', 0, 'seed', 0), totalCases, 1);

    idx = 1;
    for g = 1:numel(cfg.obstacleGroups)
        obstacleCount = cfg.obstacleGroups(g);
        for k = 1:cfg.mapsPerGroup
            scenarios(idx).caseId = idx;
            scenarios(idx).obstacleCount = obstacleCount;
            scenarios(idx).seed = cfg.randomSeed + 100 * g + 17 * k;
            idx = idx + 1;
        end
    end
end
