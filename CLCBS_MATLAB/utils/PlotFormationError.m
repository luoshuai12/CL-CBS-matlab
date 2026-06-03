function ax = PlotFormationError(errorInfo, ax)
%PLOTFORMATIONERROR Draw a readable follower formation error curve.

    if nargin < 2 || isempty(ax)
        ax = gca;
    end

    cla(ax);
    hold(ax, 'on');
    box(ax, 'off');
    grid(ax, 'off');
    fig = ancestor(ax, 'figure');
    if ~isempty(fig) && ishandle(fig)
        set(fig, 'Color', 'w');
        set(fig, 'Units', 'centimeters', 'Position', [2, 2, 8.5, 6.0]);
        set(fig, 'PaperUnits', 'centimeters', 'PaperPosition', [0, 0, 8.5, 6.0]);
    end
    set(ax, 'Units', 'normalized', 'Position', [0.16, 0.18, 0.62, 0.70], ...
        'FontName', 'Times New Roman', 'FontSize', 8, ...
        'LineWidth', 0.5, 'TickDir', 'in', 'TickLength', [0.015, 0.015], ...
        'Color', 'w');

    time = errorInfo.time;
    errors = errorInfo.errors;
    if isempty(time) || isempty(errors)
        xlabel(ax, 'Time (s)', 'FontName', 'Times New Roman', 'FontSize', 8);
        ylabel(ax, 'Tracking error (m)', 'FontName', 'Times New Roman', 'FontSize', 8);
        return;
    end

    yUpper = 50;
    xPad = max(6, 0.18 * max(1, time(end) - time(1)));

    colors = followerColors(size(errors, 2));
    lineStyles = {'-', '--', '-.', ':', '-', '--', '-.'};
    markers = {'o', 's', '^', 'd', 'v', '>'};
    markerIdx = markerIndices(numel(time));
    for k = 1:size(errors, 2)
        h = plot(ax, time, errors(:, k), ...
            'LineStyle', lineStyles{mod(k - 1, numel(lineStyles)) + 1}, ...
            'Color', colors(k, :), ...
            'LineWidth', 1.1, ...
            'Marker', markers{mod(k - 1, numel(markers)) + 1}, ...
            'MarkerSize', 3.2, ...
            'MarkerFaceColor', 'w', ...
            'DisplayName', sprintf('Agent %d', k));
        if isprop(h, 'MarkerIndices')
            h.MarkerIndices = markerIdx;
        end
    end

    meanCurve = mean(errors, 2);
    plot(ax, time, meanCurve, 'k--', ...
        'LineWidth', 1.5, ...
        'DisplayName', 'Mean');

    xlabel(ax, 'Time (s)', 'FontName', 'Times New Roman', 'FontSize', 8);
    ylabel(ax, 'Tracking error (m)', 'FontName', 'Times New Roman', 'FontSize', 8);
    xlim(ax, [time(1) - xPad, time(end) + xPad]);
    ylim(ax, [0, yUpper]);

    lgd = legend(ax, 'Location', 'northeast');
    set(lgd, 'FontName', 'Times New Roman', 'FontSize', 8, 'Box', 'off');
end

function colors = followerColors(n)
    palette = [ ...
        0.000, 0.258, 0.710; ... % blue
        0.835, 0.000, 0.000; ... % red
        0.000, 0.520, 0.120; ... % green
        0.430, 0.000, 0.620; ... % purple
        0.915, 0.380, 0.000; ... % orange
        0.000, 0.500, 0.600; ... % teal
        0.560, 0.300, 0.000];    % brown

    colors = zeros(n, 3);
    for i = 1:n
        colors(i, :) = palette(mod(i - 1, size(palette, 1)) + 1, :);
    end
end

function idx = markerIndices(n)
    if n <= 1
        idx = 1;
        return;
    end
    step = max(1, floor(n / 8));
    idx = unique([1, 1:step:n, n]);
end
