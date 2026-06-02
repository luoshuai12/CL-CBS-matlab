function style = VisualizationStyle(nAgents)
%VISUALIZATIONSTYLE Centralized high-contrast plotting style.

    if nargin < 1
        nAgents = 7;
    end

    baseColors = [ ...
        0.000, 0.258, 0.710; ... % blue
        0.835, 0.000, 0.000; ... % red
        0.000, 0.520, 0.120; ... % green
        0.430, 0.000, 0.620; ... % purple
        0.915, 0.380, 0.000; ... % orange
        0.000, 0.500, 0.600; ... % teal
        0.560, 0.300, 0.000];    % brown

    if nAgents <= size(baseColors, 1)
        colors = baseColors(1:nAgents, :);
    else
        colors = zeros(nAgents, 3);
        for i = 1:nAgents
            colors(i, :) = baseColors(mod(i - 1, size(baseColors, 1)) + 1, :);
        end
    end

    lineStyles = {'-', '--', '-.', ':', '-', '--', '-.'};
    roles = repmat({'Follower'}, 1, nAgents);
    roleNames = repmat({'Follower'}, 1, nAgents);
    for i = 1:min(3, nAgents)
        roles{i} = 'Leader';
        roleNames{i} = 'Leader';
    end

    labels = cell(1, nAgents);
    shortLabels = cell(1, nAgents);
    for i = 1:nAgents
        labels{i} = sprintf('%s%d', roleNames{i}, i - 1);
        if i <= 3
            shortLabels{i} = sprintf('L%d', i);
        else
            shortLabels{i} = sprintf('F%d', i - 3);
        end
    end

    style = struct();
    style.colors = colors;
    style.lineStyles = lineStyles;
    style.labels = labels;
    style.shortLabels = shortLabels;
    style.roles = roles;
    style.leaderIdx = 1:min(3, nAgents);
    style.followerIdx = (min(3, nAgents) + 1):nAgents;
    style.formationEdges = defaultFormationEdges(nAgents);
    style.pathLineWidth = 2.0;
    style.historyLineWidth = 0.9;
    style.historyAlpha = 0.38;
end

function edges = defaultFormationEdges(nAgents)
    template = [1, 2; 1, 3; 2, 4; 3, 5; 4, 6; 5, 7];
    keep = template(:, 1) <= nAgents & template(:, 2) <= nAgents;
    edges = template(keep, :);
end
