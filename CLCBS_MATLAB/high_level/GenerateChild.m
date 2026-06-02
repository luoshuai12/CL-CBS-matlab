function [child, success, llStats] = GenerateChild(parent, conflict, constrainedAgent, ...
    mapData, starts, goals, params, childId)
%GENERATECHILD Add one conflict constraint and replan the constrained agent.

    child = parent;
    child.id = childId;

    if constrainedAgent == conflict.agent1
        forbiddenState = conflict.state2;
        otherAgent = conflict.agent2;
    else
        forbiddenState = conflict.state1;
        otherAgent = conflict.agent1;
    end

    newConstraint = struct();
    newConstraint.time = conflict.time;
    newConstraint.state = forbiddenState;
    newConstraint.agent = otherAgent;

    child.constraints{constrainedAgent} = [child.constraints{constrainedAgent}; newConstraint];

    [plan, success, llStats] = HybridAstar(mapData, starts(constrainedAgent, :), ...
        goals(constrainedAgent, :), child.constraints{constrainedAgent}, ...
        params, constrainedAgent);

    if success
        child.solution{constrainedAgent} = plan;
        child.cost = ComputeCost(child.solution);
    end
end
