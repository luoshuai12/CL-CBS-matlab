function ax = DrawMap(mapData, starts, goals, params)
%DRAWMAP Draw map boundaries, circular obstacles, starts, and goals.

    if nargin < 4
        params = struct('carWidth', 2, 'LF', 2, 'LB', 1); %#ok<NASGU>
    end

    ax = gca;
    cla(ax);
    hold(ax, 'on');
    axis(ax, 'equal');
    xlim(ax, [0, mapData.size(1)]);
    ylim(ax, [0, mapData.size(2)]);
    grid(ax, 'on');
    box(ax, 'on');
    xlabel(ax, 'x [m]');
    ylabel(ax, 'y [m]');

    rectangle(ax, 'Position', [0, 0, mapData.size(1), mapData.size(2)], ...
        'EdgeColor', [0.15, 0.15, 0.15], 'LineWidth', 1.2);

    if isfield(mapData, 'obstacles') && ~isempty(mapData.obstacles)
        theta = linspace(0, 2 * pi, 80);
        for i = 1:size(mapData.obstacles, 1)
            obs = mapData.obstacles(i, :);
            fill(ax, obs(1) + obs(3) * cos(theta), obs(2) + obs(3) * sin(theta), ...
                [0.25, 0.25, 0.25], 'EdgeColor', 'none', 'FaceAlpha', 0.65);
        end
    end

    if nargin >= 2 && ~isempty(starts)
        plot(ax, starts(:, 1), starts(:, 2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 7);
        for i = 1:size(starts, 1)
            text(ax, starts(i, 1), starts(i, 2) + 0.8, sprintf('S%d', i - 1), ...
                'Color', [0, 0.45, 0], 'HorizontalAlignment', 'center');
        end
    end

    if nargin >= 3 && ~isempty(goals)
        plot(ax, goals(:, 1), goals(:, 2), 'rx', 'LineWidth', 1.5, 'MarkerSize', 9);
        for i = 1:size(goals, 1)
            text(ax, goals(i, 1), goals(i, 2) + 0.8, sprintf('G%d', i - 1), ...
                'Color', [0.75, 0, 0], 'HorizontalAlignment', 'center');
        end
    end
end
