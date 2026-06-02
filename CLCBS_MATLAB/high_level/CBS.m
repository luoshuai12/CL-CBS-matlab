function [solution, success, stats] = CBS(mapData, starts, goals, params)
%CBS High-level Conflict-Based Search for car-like robots.

    nAgents = size(starts, 1);
    stats = struct('cost', inf, 'makespan', inf, 'highLevelExpanded', 0, ...
        'lowLevelExpanded', 0, 'conflictsResolved', 0);

    root = emptyNode(nAgents);
    root.id = 1;
    root.cost = 0;

    for agentIdx = 1:nAgents
        [plan, ok, llStats] = HybridAstar(mapData, starts(agentIdx, :), ...
            goals(agentIdx, :), root.constraints{agentIdx}, params, agentIdx);
        stats.lowLevelExpanded = stats.lowLevelExpanded + llStats.expanded;
        if ~ok
            solution = {};
            success = false;
            return;
        end
        root.solution{agentIdx} = plan;
    end
    root.cost = ComputeCost(root.solution);

    open = root;
    nextId = 2;

    while ~isempty(open) && stats.highLevelExpanded < params.maxHighLevelIterations
        [~, order] = sortrows([[open.cost].', [open.id].'], [1, 2]);
        open = open(order(:, 1));
        node = open(1);
        open(1) = [];

        stats.highLevelExpanded = stats.highLevelExpanded + 1;
        conflict = DetectConflict(node.solution, params);
        if ~conflict.hasConflict
            solution = node.solution;
            success = true;
            stats.cost = ComputeCost(solution);
            stats.makespan = computeMakespan(solution);
            return;
        end

        stats.conflictsResolved = stats.conflictsResolved + 1;
        childAgents = [conflict.agent1, conflict.agent2];
        for k = 1:numel(childAgents)
            [child, ok, llStats] = GenerateChild(node, conflict, childAgents(k), ...
                mapData, starts, goals, params, nextId);
            nextId = nextId + 1;
            stats.lowLevelExpanded = stats.lowLevelExpanded + llStats.expanded;
            if ok
                open(end + 1) = child; %#ok<AGROW>
            end
        end
    end

    solution = {};
    success = false;
end

function node = emptyNode(nAgents)
    node = struct();
    node.solution = cell(1, nAgents);
    node.constraints = cell(1, nAgents);
    for i = 1:nAgents
        node.constraints{i} = repmat(emptyConstraint(), 0, 1);
    end
    node.cost = inf;
    node.id = 0;
end

function c = emptyConstraint()
    c = struct('time', 0, 'state', zeros(1, 4), 'agent', 0);
end

function makespan = computeMakespan(solution)
    makespan = 0;
    for i = 1:numel(solution)
        if ~isempty(solution{i}.states)
            makespan = max(makespan, solution{i}.states(end, 4));
        end
    end
end
