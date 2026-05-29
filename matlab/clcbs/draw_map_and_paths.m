function draw_map_and_paths(mapData, paths, cfg)
% draw_map_and_paths
% Draw circular obstacles and all robot paths.

    hold on;
    axis equal;
    xlim([0, cfg.mapSize(1)]);
    ylim([0, cfg.mapSize(2)]);
    grid on;
    xlabel('x (m)');
    ylabel('y (m)');

    theta = linspace(0, 2 * pi, 60);
    for i = 1:size(mapData.obstacles, 1)
        obs = mapData.obstacles(i, :);
        fill(obs(1) + obs(3) * cos(theta), obs(2) + obs(3) * sin(theta), ...
            [0.3, 0.3, 0.3], 'FaceAlpha', 0.25, 'EdgeColor', [0.2, 0.2, 0.2]);
    end

    cmap = lines(numel(paths));
    for i = 1:numel(paths)
        p = paths{i};
        plot(p(:, 1), p(:, 2), '-', 'Color', cmap(i, :), 'LineWidth', 1.7);
        plot(p(1, 1), p(1, 2), 'o', 'Color', cmap(i, :), 'MarkerFaceColor', cmap(i, :), 'MarkerSize', 5);
        plot(p(end, 1), p(end, 2), 's', 'Color', cmap(i, :), 'MarkerFaceColor', 'w', 'MarkerSize', 5);
    end

    hold off;
end
