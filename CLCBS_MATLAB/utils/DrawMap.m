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
        safetyRadius = 0;
        if nargin >= 4 && isstruct(params)
            safetyRadius = hypot(params.carWidth / 2, (params.LF + params.LB) / 2);
        end
        for i = 1:size(mapData.obstacles, 1)
            obs = mapData.obstacles(i, :);
            safeR = obs(3) + safetyRadius;
            fill(ax, obs(1) + safeR * cos(theta), obs(2) + safeR * sin(theta), ...
                [0.95, 0.35, 0.15], 'EdgeColor', [0.85, 0.25, 0.05], ...
                'LineStyle', '--', 'LineWidth', 0.8, 'FaceAlpha', 0.12, ...
                'HandleVisibility', 'off');
            fill(ax, obs(1) + obs(3) * cos(theta), obs(2) + obs(3) * sin(theta), ...
                [0.30, 0.30, 0.30], 'EdgeColor', [0.05, 0.05, 0.05], ...
                'LineWidth', 1.1, 'FaceAlpha', 0.48, 'HandleVisibility', 'off');
        end
    end

    if nargin >= 2 && ~isempty(starts)
        for i = 1:size(starts, 1)
            drawVehicleFrame(ax, starts(i, :), params, [0.0, 0.55, 0.1], '-', 1.1, 0.08);
        end
    end

    if nargin >= 3 && ~isempty(goals)
        for i = 1:size(goals, 1)
            drawVehicleFrame(ax, goals(i, :), params, [0.0, 0.55, 0.1], '--', 1.2, 0.03);
        end
    end
end

function drawVehicleFrame(ax, state, params, color, lineStyle, lineWidth, faceAlpha)
    corners = vehicleCorners(state, params);
    patch(ax, corners(:, 1), corners(:, 2), color, ...
        'FaceAlpha', faceAlpha, ...
        'EdgeColor', color, ...
        'LineStyle', lineStyle, ...
        'LineWidth', lineWidth, ...
        'HandleVisibility', 'off');
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
