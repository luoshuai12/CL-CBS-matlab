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
        DrawVehicle(states(1, :), params, colors(i, :), 0.18, ax);
        DrawVehicle(states(end, :), params, colors(i, :), 0.35, ax);
    end
    labels = arrayfun(@(idx) sprintf('agent %d', idx), 1:numel(solution), 'UniformOutput', false);
    legend(ax, handles, labels, 'Location', 'bestoutside');
end
