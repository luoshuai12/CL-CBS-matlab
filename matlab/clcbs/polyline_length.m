function len = polyline_length(path)
% polyline_length
% Sum segment lengths of a polyline path.

    if size(path, 1) < 2
        len = 0;
        return;
    end
    len = sum(vecnorm(diff(path, 1, 1), 2, 2));
end
