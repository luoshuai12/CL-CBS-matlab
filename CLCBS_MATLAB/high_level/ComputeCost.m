function cost = ComputeCost(solution)
%COMPUTECOST Sum-of-costs objective used by CBS high-level ordering.

    cost = 0;
    for i = 1:numel(solution)
        if isempty(solution{i})
            cost = inf;
            return;
        end
        cost = cost + solution{i}.cost;
    end
end
