function plot_formation_layout(cfg)
% plot_formation_layout
% Save initial formation layout figure (like "图 5 编队队形").

    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [120 120 900 350]);
    hold on;
    axis equal;
    xlim([0, cfg.mapSize(1)]);
    ylim([0, cfg.mapSize(2)]);
    grid on;
    xlabel('x (m)');
    ylabel('y (m)');
    title('图 5 编队队形');

    starts = cfg.startStates(:, 1:2);
    for i = 1:size(starts, 1)
        draw_robot_rect(starts(i, :), cfg.robotSize, sprintf('%d', i - 1));
    end

    hold off;
    saveas(fig, fullfile(cfg.resultDir, 'formation_layout.png'));
    close(fig);
end

function draw_robot_rect(center, robotSize, labelText)
    x = center(1);
    y = center(2);
    w = robotSize(2); % length
    h = robotSize(1); % width

    rectangle('Position', [x - w / 2, y - h / 2, w, h], ...
        'EdgeColor', [0.25, 0.25, 0.25], 'LineWidth', 2.1, 'FaceColor', [0.72, 0.72, 0.72]);
    text(x, y, labelText, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'FontSize', 13);
end
