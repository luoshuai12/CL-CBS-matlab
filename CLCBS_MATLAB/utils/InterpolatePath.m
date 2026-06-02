function dense = InterpolatePath(path, step)
%INTERPOLATEPATH Resample a path [x y yaw t] at approximately uniform spacing.

    if nargin < 2 || isempty(step)
        step = 0.5;
    end

    if isempty(path)
        dense = zeros(0, 4);
        return;
    end

    dense = path(1, :);
    for i = 2:size(path, 1)
        a = path(i - 1, :);
        b = path(i, :);
        dist = hypot(b(1) - a(1), b(2) - a(2));
        n = max(1, ceil(dist / step));
        yawDelta = wrapToPiLocal(b(3) - a(3));
        for k = 1:n
            alpha = k / n;
            sample = zeros(1, 4);
            sample(1:2) = a(1:2) + alpha * (b(1:2) - a(1:2));
            sample(3) = mod(a(3) + alpha * yawDelta, 2 * pi);
            sample(4) = a(4) + alpha * (b(4) - a(4));
            dense(end + 1, :) = sample; %#ok<AGROW>
        end
    end
end

function angle = wrapToPiLocal(angle)
    angle = mod(angle + pi, 2 * pi) - pi;
end
