function handles = DrawTrajectory(solution, params)
%DRAWTRAJECTORY Overlay planned agent paths and terminal footprints.

    if isempty(solution)
        handles = [];
        return;
    end

    ax = gca;
    hold(ax, 'on');
    style = VisualizationStyle(numel(solution));
    colors = style.colors;
    handles = zeros(numel(solution), 1);

    for i = 1:numel(solution)
        states = solution{i}.states;
        lineStyle = style.lineStyles{mod(i - 1, numel(style.lineStyles)) + 1};
        handles(i) = plot(ax, states(:, 1), states(:, 2), lineStyle, ...
            'Color', colors(i, :), 'LineWidth', style.pathLineWidth, ...
            'DisplayName', style.labels{i});
        plot(ax, states(:, 1), states(:, 2), '.', 'Color', colors(i, :), ...
            'MarkerSize', 6, 'HandleVisibility', 'off');
        DrawVehicle(states(1, :), params, colors(i, :), 0.18, ax);
        DrawVehicle(states(end, :), params, colors(i, :), 0.35, ax);
    end

    drawFormationEdges(ax, solution, style);
    legend(ax, handles, style.labels, 'Location', 'eastoutside');
end

function drawFormationEdges(ax, solution, style)
    edges = style.formationEdges;
    if isempty(edges)
        return;
    end
    for k = 1:size(edges, 1)
        a = edges(k, 1);
        b = edges(k, 2);
        if isempty(solution{a}.states) || isempty(solution{b}.states)
            continue;
        end
        p1 = solution{a}.states(end, 1:2);
        p2 = solution{b}.states(end, 1:2);
        plot(ax, [p1(1), p2(1)], [p1(2), p2(2)], 'k--', ...
            'LineWidth', 1.0, 'Color', [0.1, 0.1, 0.1], ...
            'HandleVisibility', 'off');
    end
end
