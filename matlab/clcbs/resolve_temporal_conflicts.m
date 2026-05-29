function paths = resolve_temporal_conflicts(paths, minDistance, maxIter)
% resolve_temporal_conflicts
% Resolve close-proximity conflicts by inserting waits.

    for iter = 1:maxIter
        [hasConflict, delayRobot, delayStep] = detect_first_conflict(paths, minDistance);
        if ~hasConflict
            return;
        end

        p = paths{delayRobot};
        insertIdx = max(2, min(delayStep, size(p, 1)));
        holdPoint = p(insertIdx - 1, :);
        p = [p(1:insertIdx - 1, :); holdPoint; p(insertIdx:end, :)];
        paths{delayRobot} = p;
    end
end
