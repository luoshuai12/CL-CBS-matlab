function handles = DrawTrajectory(solution, params)
%DRAWTRAJECTORY Overlay planned agent paths and terminal footprints.

    if isempty(solution)
        handles = [];
        return;
    end

    ax = gca;
    hold(ax, 'on');
    colors = lines(numel(solution));
    handles = zeros(numel(solution), 1);

    for i = 1:numel(solution)
        states = solution{i}.states;
        handles(i) = plot(ax, states(:, 1), states(:, 2), '-', ...
            'Color', colors(i, :), 'LineWidth', 1.8);
        plot(ax, states(:, 1), states(:, 2), '.', 'Color', colors(i, :), 'MarkerSize', 7);
        drawFootprint(ax, states(1, :), params, colors(i, :), 0.18);
        drawFootprint(ax, states(end, :), params, colors(i, :), 0.35);
    end
    labels = arrayfun(@(idx) sprintf('agent %d', idx), 1:numel(solution), 'UniformOutput', false);
    legend(ax, handles, labels, 'Location', 'bestoutside');
end

function drawFootprint(ax, state, params, color, alpha)
    corners = vehicleCorners(state, params);
    patch(ax, corners(:, 1), corners(:, 2), color, ...
        'FaceAlpha', alpha, 'EdgeColor', color, 'LineWidth', 1.0);
end

function corners = vehicleCorners(state, params)
    halfW = params.carWidth / 2;
    local = [ ...
        params.LF,  halfW; ...
        params.LF, -halfW; ...
       -params.LB, -halfW; ...
       -params.LB,  halfW];
    yaw = state(3);
    rot = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
    corners = local * rot.' + state(1:2);
end
