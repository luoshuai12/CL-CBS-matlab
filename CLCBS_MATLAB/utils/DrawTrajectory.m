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
        fadedColor = fadeColor(colors(i, :), style.historyAlpha);
        handles(i) = plot(ax, states(:, 1), states(:, 2), lineStyle, ...
            'Color', fadedColor, 'LineWidth', style.historyLineWidth, ...
            'HandleVisibility', 'off');
        plot(ax, states(:, 1), states(:, 2), '.', 'Color', fadedColor, ...
            'MarkerSize', 6, 'HandleVisibility', 'off');
        DrawVehicle(states(end, :), params, colors(i, :), 0.62, ax);
        plot(ax, states(end, 1), states(end, 2), 'o', 'Color', colors(i, :), ...
            'MarkerFaceColor', colors(i, :), 'MarkerSize', 5, ...
            'HandleVisibility', 'off');
        text(ax, states(end, 1) + 1.2, states(end, 2) + 0.8, style.shortLabels{i}, ...
            'Color', colors(i, :), 'FontWeight', 'bold', 'FontSize', 9, ...
            'HandleVisibility', 'off');
    end

    drawFormationEdges(ax, solution, style);
    try
        legend(ax, 'off');
    catch
    end
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

function c = fadeColor(color, alpha)
    c = 1 - alpha * (1 - color);
end
