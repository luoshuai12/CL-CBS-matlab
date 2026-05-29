function synced = synchronize_paths(paths)
% synchronize_paths
% Pad shorter paths with terminal states to equalize timeline length.

    n = numel(paths);
    pathLens = zeros(n, 1);
    for i = 1:n
        pathLens(i) = size(paths{i}, 1);
    end
    maxLen = max(pathLens);

    synced = cell(n, 1);
    for i = 1:n
        p = paths{i};
        if size(p, 1) < maxLen
            p = [p; repmat(p(end, :), maxLen - size(p, 1), 1)];
        end
        synced{i} = p;
    end
end
