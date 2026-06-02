function errorInfo = ComputeError(solution, starts, leaderIdx, followerIdx)
%COMPUTEERROR Compute follower formation errors relative to a leader.
%   Desired offsets are inferred from the initial start configuration.

    if nargin < 3 || isempty(leaderIdx)
        leaderIdx = 1;
    end
    if nargin < 4 || isempty(followerIdx)
        followerIdx = setdiff(1:numel(solution), leaderIdx);
    end

    if isempty(solution)
        errorInfo = emptyErrorInfo();
        return;
    end

    desiredOffsets = starts(:, 1:2) - starts(leaderIdx, 1:2);
    maxT = 0;
    for i = 1:numel(solution)
        maxT = max(maxT, solution{i}.states(end, 4));
    end

    time = (0:maxT).';
    errors = zeros(numel(time), numel(followerIdx));

    for tIdx = 1:numel(time)
        t = time(tIdx);
        leaderState = stateAtTime(solution{leaderIdx}.states, t);
        for k = 1:numel(followerIdx)
            agentIdx = followerIdx(k);
            followerState = stateAtTime(solution{agentIdx}.states, t);
            desiredPosition = leaderState(1:2) + desiredOffsets(agentIdx, :);
            errors(tIdx, k) = norm(followerState(1:2) - desiredPosition);
        end
    end

    errorInfo = struct();
    errorInfo.time = time;
    errorInfo.errors = errors;
    errorInfo.meanError = mean(errors(:));
    errorInfo.maxError = max(errors(:));
    errorInfo.leaderIdx = leaderIdx;
    errorInfo.followerIdx = followerIdx;
    errorInfo.desiredOffsets = desiredOffsets;
end

function errorInfo = emptyErrorInfo()
    errorInfo = struct('time', zeros(0, 1), 'errors', zeros(0, 0), ...
        'meanError', NaN, 'maxError', NaN, 'leaderIdx', 1, ...
        'followerIdx', [], 'desiredOffsets', zeros(0, 2));
end

function state = stateAtTime(states, t)
    idx = find(states(:, 4) >= t, 1, 'first');
    if isempty(idx)
        state = states(end, :);
        state(4) = t;
    else
        state = states(idx, :);
        state(4) = t;
    end
end
