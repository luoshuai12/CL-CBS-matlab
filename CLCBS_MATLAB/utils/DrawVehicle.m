function h = DrawVehicle(state, params, color, faceAlpha, ax)
%DRAWVEHICLE Draw a car-like robot footprint at [x y yaw t].

    if nargin < 3 || isempty(color)
        color = [0.1, 0.4, 0.9];
    end
    if nargin < 4 || isempty(faceAlpha)
        faceAlpha = 0.35;
    end
    if nargin < 5 || isempty(ax)
        ax = gca;
    end

    corners = vehicleCorners(state, params);
    h.body = patch(ax, corners(:, 1), corners(:, 2), color, ...
        'FaceAlpha', faceAlpha, 'EdgeColor', color, 'LineWidth', 1.1, ...
        'HandleVisibility', 'off');

    front = mean(corners(1:2, :), 1);
    rear = mean(corners(3:4, :), 1);
    h.heading = plot(ax, [rear(1), front(1)], [rear(2), front(2)], ...
        '-', 'Color', color, 'LineWidth', 1.4, 'HandleVisibility', 'off');
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
