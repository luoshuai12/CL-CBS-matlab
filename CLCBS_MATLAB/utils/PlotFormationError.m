function ax = PlotFormationError(errorInfo, ax)
%PLOTFORMATIONERROR Draw a readable follower formation error curve.

    if nargin < 2 || isempty(ax)
        ax = gca;
    end

    cla(ax);
    hold(ax, 'on');
    box(ax, 'on');
    grid(ax, 'on');
    fig = ancestor(ax, 'figure');
    if ~isempty(fig) && ishandle(fig)
        set(fig, 'Position', [100, 100, 1120, 520]);
    end
    set(ax, 'Units', 'normalized', 'Position', [0.08, 0.20, 0.74, 0.58]);
    ax.GridAlpha = 0.18;
    ax.MinorGridAlpha = 0.08;

    time = errorInfo.time;
    errors = errorInfo.errors;
    if isempty(time) || isempty(errors)
        title(ax, 'Follower Formation Error');
        return;
    end

    yUpper = 50;
    xPad = max(6, 0.18 * max(1, time(end) - time(1)));

    meanCurve = mean(errors, 2);
    highlightHighErrorPhase(ax, time, meanCurve, yUpper);

    colors = followerColors(size(errors, 2));
    for k = 1:size(errors, 2)
        plot(ax, time, errors(:, k), '-', ...
            'Color', colors(k, :), ...
            'LineWidth', 1.6, ...
            'DisplayName', sprintf('F%d', k));
    end

    plot(ax, time, meanCurve, 'k--', ...
        'LineWidth', 2.6, ...
        'DisplayName', 'Mean');

    xlabel(ax, 'time step');
    ylabel(ax, 'formation error [m]');
    title(ax, 'Follower Formation Error');
    xlim(ax, [time(1) - xPad, time(end) + xPad]);
    ylim(ax, [0, yUpper]);

    legend(ax, 'Location', 'eastoutside');
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

function highlightHighErrorPhase(ax, time, meanCurve, yUpper)
    if isempty(meanCurve) || max(meanCurve) <= 0
        return;
    end

    threshold = 0.45 * max(meanCurve);
    active = meanCurve >= threshold;
    if ~any(active)
        return;
    end

    yLimit = [0, yUpper];
    idx = find(active);
    breaks = [1; find(diff(idx) > 1) + 1; numel(idx) + 1];
    for b = 1:numel(breaks) - 1
        segment = idx(breaks(b):breaks(b + 1) - 1);
        x1 = time(segment(1));
        x2 = time(segment(end));
        patch(ax, [x1, x2, x2, x1], [yLimit(1), yLimit(1), yLimit(2), yLimit(2)], ...
            [0.88, 0.88, 0.88], ...
            'FaceAlpha', 0.25, ...
            'EdgeColor', 'none', ...
            'HandleVisibility', 'off');
    end
end
