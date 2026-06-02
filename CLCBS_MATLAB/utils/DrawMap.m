function ax = DrawMap(mapData, starts, goals, params)
%DRAWMAP Draw map boundaries, circular obstacles, starts, and goals.

    if nargin < 4
        params = struct('carWidth', 2, 'LF', 2, 'LB', 1); %#ok<NASGU>
    end

    ax = gca;
    cla(ax);
    try
        legend(ax, 'off');
    catch
    end
    hold(ax, 'on');
    axis(ax, 'equal');
    xlim(ax, [0, mapData.size(1)]);
    ylim(ax, [0, mapData.size(2)]);
    grid(ax, 'on');
    box(ax, 'on');
    xlabel(ax, 'x [m]');
    ylabel(ax, 'y [m]');

    rectangle(ax, 'Position', [0, 0, mapData.size(1), mapData.size(2)], ...
        'EdgeColor', [0.15, 0.15, 0.15], 'LineWidth', 1.2, ...
        'HandleVisibility', 'off');

    if isfield(mapData, 'obstacles') && ~isempty(mapData.obstacles)
        theta = linspace(0, 2 * pi, 80);
        for i = 1:size(mapData.obstacles, 1)
            obs = mapData.obstacles(i, :);
            fill(ax, obs(1) + obs(3) * cos(theta), obs(2) + obs(3) * sin(theta), ...
                [0.30, 0.30, 0.30], 'EdgeColor', [0.05, 0.05, 0.05], ...
                'LineWidth', 1.1, 'FaceAlpha', 0.48, 'HandleVisibility', 'off');
        end
    end

    if nargin >= 2 && ~isempty(starts)
        plot(ax, starts(:, 1), starts(:, 2), 'go', 'MarkerFaceColor', [0, 0.8, 0], ...
            'MarkerSize', 7, 'HandleVisibility', 'off');
        for i = 1:size(starts, 1)
            labelPrefix = rolePrefix(i);
            alphaColor = labelColor(i);
            text(ax, starts(i, 1), starts(i, 2) + 0.8, sprintf('%s%d', labelPrefix, i - 1), ...
                'Color', alphaColor, 'FontSize', labelSize(i), ...
                'HorizontalAlignment', 'center', 'HandleVisibility', 'off');
        end
    end

    if nargin >= 3 && ~isempty(goals)
        plot(ax, goals(:, 1), goals(:, 2), 'rx', 'LineWidth', 1.7, ...
            'MarkerSize', 9, 'HandleVisibility', 'off');
        for i = 1:size(goals, 1)
            labelPrefix = rolePrefix(i);
            alphaColor = goalLabelColor(i);
            text(ax, goals(i, 1), goals(i, 2) + 0.8, sprintf('%s-G%d', labelPrefix, i - 1), ...
                'Color', alphaColor, 'FontSize', labelSize(i), ...
                'HorizontalAlignment', 'center', 'HandleVisibility', 'off');
        end
    end
end

function prefix = rolePrefix(idx)
    if idx <= 3
        prefix = 'L';
    else
        prefix = 'F';
    end
end

function sz = labelSize(idx)
    if idx <= 3
        sz = 10;
    else
        sz = 8;
    end
end

function c = labelColor(idx)
    if idx <= 3
        c = [0, 0.45, 0];
    else
        c = [0.15, 0.55, 0.15];
    end
end

function c = goalLabelColor(idx)
    if idx <= 3
        c = [0.78, 0, 0];
    else
        c = [0.85, 0.2, 0.2];
    end
end
