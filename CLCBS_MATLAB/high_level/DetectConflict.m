function conflict = DetectConflict(solution, params)
%DETECTCONFLICT Return the first body conflict among all agents.

    nAgents = numel(solution);
    maxTime = 0;
    for i = 1:nAgents
        if ~isempty(solution{i}.states)
            maxTime = max(maxTime, solution{i}.states(end, 4));
        end
    end

    conflict = struct('hasConflict', false, 'time', -1, 'agent1', 0, ...
        'agent2', 0, 'state1', [], 'state2', []);

    for t = 0:maxTime
        for i = 1:nAgents
            state1 = stateAtTime(solution{i}, t);
            for j = (i + 1):nAgents
                state2 = stateAtTime(solution{j}, t);
                if CollisionCheck(state1, state2, params)
                    conflict.hasConflict = true;
                    conflict.time = t;
                    conflict.agent1 = i;
                    conflict.agent2 = j;
                    conflict.state1 = state1;
                    conflict.state2 = state2;
                    return;
                end
            end
        end
    end
end

function state = stateAtTime(plan, t)
    states = plan.states;
    idx = find(states(:, 4) >= t, 1, 'first');
    if isempty(idx)
        state = states(end, :);
        state(4) = t;
    else
        state = states(idx, :);
        state(4) = t;
    end
end
