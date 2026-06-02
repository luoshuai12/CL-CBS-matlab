function valid = CheckConstraint(state, constraints, params)
%CHECKCONSTRAINT Test a state against CBS body constraints.

    valid = true;
    if isempty(constraints)
        return;
    end

    for i = 1:numel(constraints)
        c = constraints(i);
        if state(4) < c.time || state(4) > c.time + params.constraintWaitTime
            continue;
        end

        cState = c.state;
        if numel(cState) < 4
            cState(4) = c.time;
        else
            cState(4) = c.time;
        end

        if CollisionCheck(state, cState, params)
            valid = false;
            return;
        end
    end
end
