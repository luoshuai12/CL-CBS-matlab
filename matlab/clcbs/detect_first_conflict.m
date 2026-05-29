function [hasConflict, delayRobot, delayStep] = detect_first_conflict(paths, minDistance)
% detect_first_conflict
% Detect first collision or edge-swap conflict in synchronized trajectories.

    hasConflict = false;
    delayRobot = 0;
    delayStep = 0;

    synced = synchronize_paths(paths);
    nRobot = numel(synced);
    nStep = size(synced{1}, 1);

    for t = 2:nStep
        for i = 1:nRobot - 1
            pi = synced{i}(t, :);
            piPrev = synced{i}(t - 1, :);
            for j = i + 1:nRobot
                pj = synced{j}(t, :);
                pjPrev = synced{j}(t - 1, :);

                if norm(pi - pj) < minDistance
                    hasConflict = true;
                    delayRobot = max(i, j);
                    delayStep = t;
                    return;
                end

                edgeSwap = norm(piPrev - pj) < minDistance && norm(pjPrev - pi) < minDistance;
                if edgeSwap
                    hasConflict = true;
                    delayRobot = max(i, j);
                    delayStep = t;
                    return;
                end
            end
        end
    end
end
