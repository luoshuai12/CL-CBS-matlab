function gui_main()
%GUI_MAIN MATLAB GUI for configuring and visualizing the CL-CBS demo.

    rootDir = fileparts(mfilename('fullpath'));
    activateProjectPath(rootDir);

    app = struct();
    app.rootDir = rootDir;
    app.results = [];

    fig = figure('Name', 'CL-CBS MATLAB GUI', 'NumberTitle', 'off', ...
        'MenuBar', 'none', 'ToolBar', 'figure', 'Position', [100, 100, 1100, 680]);

    app.ax = axes('Parent', fig, 'Units', 'normalized', ...
        'Position', [0.33, 0.15, 0.64, 0.78]);
    grid(app.ax, 'on');
    axis(app.ax, 'equal');
    title(app.ax, 'CL-CBS simulation');

    panel = uipanel('Parent', fig, 'Title', '参数设置 / 控制', ...
        'Units', 'normalized', 'Position', [0.02, 0.15, 0.28, 0.78]);

    uicontrol(panel, 'Style', 'text', 'String', '障碍物数量', ...
        'HorizontalAlignment', 'left', 'Units', 'normalized', ...
        'Position', [0.06, 0.88, 0.42, 0.05]);
    app.obstaclePopup = uicontrol(panel, 'Style', 'popupmenu', ...
        'String', {'10', '20', '30'}, 'Value', 1, ...
        'Units', 'normalized', 'Position', [0.52, 0.88, 0.38, 0.055], ...
        'Callback', @refreshInitialMap);

    uicontrol(panel, 'Style', 'text', 'String', '最大高层迭代', ...
        'HorizontalAlignment', 'left', 'Units', 'normalized', ...
        'Position', [0.06, 0.80, 0.42, 0.05]);
    app.maxHighLevelEdit = uicontrol(panel, 'Style', 'edit', 'String', '240', ...
        'Units', 'normalized', 'Position', [0.52, 0.80, 0.38, 0.055]);

    uicontrol(panel, 'Style', 'text', 'String', '最大低层节点', ...
        'HorizontalAlignment', 'left', 'Units', 'normalized', ...
        'Position', [0.06, 0.72, 0.42, 0.05]);
    app.maxLowLevelEdit = uicontrol(panel, 'Style', 'edit', 'String', '90000', ...
        'Units', 'normalized', 'Position', [0.52, 0.72, 0.38, 0.055]);

    uicontrol(panel, 'Style', 'text', 'String', '目标容差(m)', ...
        'HorizontalAlignment', 'left', 'Units', 'normalized', ...
        'Position', [0.06, 0.64, 0.42, 0.05]);
    app.goalToleranceEdit = uicontrol(panel, 'Style', 'edit', 'String', '2.5', ...
        'Units', 'normalized', 'Position', [0.52, 0.64, 0.38, 0.055]);

    uicontrol(panel, 'Style', 'text', 'String', '动画步长(s)', ...
        'HorizontalAlignment', 'left', 'Units', 'normalized', ...
        'Position', [0.06, 0.56, 0.42, 0.05]);
    app.pauseEdit = uicontrol(panel, 'Style', 'edit', 'String', '0.05', ...
        'Units', 'normalized', 'Position', [0.52, 0.56, 0.38, 0.055]);

    app.runButton = uicontrol(panel, 'Style', 'pushbutton', 'String', '运行仿真', ...
        'Units', 'normalized', 'Position', [0.08, 0.46, 0.36, 0.07], ...
        'Callback', @runSimulation);
    app.playButton = uicontrol(panel, 'Style', 'pushbutton', 'String', '动画播放', ...
        'Units', 'normalized', 'Position', [0.54, 0.46, 0.36, 0.07], ...
        'Callback', @playAnimation);
    app.clearButton = uicontrol(panel, 'Style', 'pushbutton', 'String', '清空显示', ...
        'Units', 'normalized', 'Position', [0.08, 0.38, 0.82, 0.06], ...
        'Callback', @clearDisplay);

    app.statusText = uicontrol(panel, 'Style', 'listbox', 'String', {'等待运行...'}, ...
        'Units', 'normalized', 'Position', [0.06, 0.05, 0.88, 0.35], ...
        'Max', 2, 'Min', 0);

    guidata(fig, app);
    drawInitialMap(fig);
end

function activateProjectPath(rootDir)
    addpath(rootDir, '-begin');
    addpath(fullfile(rootDir, 'map'), '-begin');
    addpath(fullfile(rootDir, 'high_level'), '-begin');
    addpath(fullfile(rootDir, 'low_level'), '-begin');
    addpath(fullfile(rootDir, 'utils'), '-begin');
    clear CLCBS_RunSimulation CreateMap CBS HybridAstar DubinsHeuristic DubinsPlanner CheckConstraint;
    clear DetectConflict GenerateChild ComputeCost ComputeError CollisionCheck;
    clear DefaultScenario DrawMap DrawTrajectory DrawVehicle InterpolatePath;
    rehash;
end

function runSimulation(src, ~)
    fig = ancestor(src, 'figure');
    app = guidata(fig);
    setStatus(app, {'运行中，请稍候...'});
    drawnow;

    cfg = struct();
    cfg.showFigure = false;
    cfg.saveResults = true;
    cfg.forceGenerate = true;
    cfg.mapSize = [150, 50];
    cfg.obstacleCount = selectedObstacleCount(app);
    cfg.randomSeed = 20260602 + cfg.obstacleCount;
    cfg.params = struct();
    cfg.params.maxHighLevelIterations = readScalar(app.maxHighLevelEdit, 240);
    cfg.params.maxLowLevelNodes = readScalar(app.maxLowLevelEdit, 90000);
    cfg.params.goalTolerance = readScalar(app.goalToleranceEdit, 2.5);

    try
        activateProjectPath(app.rootDir);
        cfg.outputDir = fullfile(app.rootDir, 'result', 'save_results');
        app.results = CLCBS_RunSimulation(cfg);
        guidata(fig, app);
        cla(app.ax);
        axes(app.ax); %#ok<LAXES>
        DrawMap(app.results.map, app.results.starts, app.results.goals, app.results.params);
        if app.results.success
            DrawTrajectory(app.results.solution, app.results.params);
            plotErrorCurve(app.results);
        end
        setStatus(app, buildStatusLines(app.results));
    catch err
        setStatus(app, buildErrorLines(err));
    end
end

function playAnimation(src, ~)
    fig = ancestor(src, 'figure');
    app = guidata(fig);
    if isempty(app.results) || ~app.results.success
        setStatus(app, {'请先成功运行仿真。'});
        return;
    end

    pauseTime = readScalar(app.pauseEdit, 0.05);
    results = app.results;
    maxT = 0;
    for i = 1:numel(results.solution)
        maxT = max(maxT, results.solution{i}.states(end, 4));
    end

    cla(app.ax);
    axes(app.ax); %#ok<LAXES>
    DrawMap(results.map, results.starts, results.goals, results.params);
    colors = lines(numel(results.solution));
    vehicleHandles = repmat(struct('body', [], 'heading', []), numel(results.solution), 1);
    for i = 1:numel(results.solution)
        plot(app.ax, results.solution{i}.states(:, 1), results.solution{i}.states(:, 2), ...
            ':', 'Color', colors(i, :), 'LineWidth', 1.0);
        vehicleHandles(i) = DrawVehicle(stateAtTime(results.solution{i}.states, 0), ...
            results.params, colors(i, :), 0.55, app.ax);
    end

    for t = 0:maxT
        for i = 1:numel(results.solution)
            state = stateAtTime(results.solution{i}.states, t);
            [corners, rear, front] = vehicleGeometry(state, results.params);
            set(vehicleHandles(i).body, 'XData', corners(:, 1), 'YData', corners(:, 2));
            set(vehicleHandles(i).heading, 'XData', [rear(1), front(1)], ...
                'YData', [rear(2), front(2)]);
        end
        title(app.ax, sprintf('CL-CBS animation, t = %d', t));
        try
            drawnow limitrate;
        catch
            drawnow;
        end
        pause(pauseTime);
    end
end

function clearDisplay(src, ~)
    fig = ancestor(src, 'figure');
    app = guidata(fig);
    cla(app.ax);
    drawInitialMap(fig);
    app.results = [];
    guidata(fig, app);
    setStatus(app, {'已清空。'});
end

function refreshInitialMap(src, ~)
    fig = ancestor(src, 'figure');
    drawInitialMap(fig);
end

function drawInitialMap(fig)
    app = guidata(fig);
    config = struct();
    config.mapSize = [150, 50];
    config.obstacleCount = selectedObstacleCount(app);
    config.randomSeed = 20260602 + selectedObstacleCount(app);
    [mapData, starts, goals, params] = CreateMap(config);
    cla(app.ax);
    axes(app.ax); %#ok<LAXES>
    DrawMap(mapData, starts, goals, params);
    title(app.ax, sprintf('七机器人随机障碍物地图: %d obstacles', size(mapData.obstacles, 1)));
end

function value = readScalar(handle, defaultValue)
    value = str2double(get(handle, 'String'));
    if ~isfinite(value)
        value = defaultValue;
    end
end

function lines = buildStatusLines(results)
    if results.success
        lines = { ...
            '运行成功', ...
            sprintf('地图大小: %.0f x %.0f m', results.map.size(1), results.map.size(2)), ...
            sprintf('机器人数量: %d', size(results.starts, 1)), ...
            sprintf('障碍物数量: %d', size(results.map.obstacles, 1)), ...
            sprintf('机器人尺寸: 1m x 2m'), ...
            sprintf('路径总代价: %.3f', results.stats.cost), ...
            sprintf('最大完成时刻: %d', results.stats.makespan), ...
            sprintf('高层扩展节点: %d', results.stats.highLevelExpanded), ...
            sprintf('低层扩展节点: %d', results.stats.lowLevelExpanded), ...
            sprintf('运行时间: %.3f s', results.stats.runtime), ...
            sprintf('平均跟随误差: %.3f m', results.error.meanError), ...
            ['结果目录: ', results.outputDir]};
    else
        lines = { ...
            '未找到无冲突路径', ...
            sprintf('高层扩展节点: %d', results.stats.highLevelExpanded), ...
            sprintf('低层扩展节点: %d', results.stats.lowLevelExpanded), ...
            sprintf('运行时间: %.3f s', results.stats.runtime)};
    end
end

function plotErrorCurve(results)
    if ~isfield(results, 'error') || isempty(results.error.time) || isempty(results.error.errors)
        return;
    end

    fig = figure('Name', 'Formation Error Curve', 'NumberTitle', 'off');
    ax = axes('Parent', fig);
    hold(ax, 'on');
    grid(ax, 'on');

    for k = 1:size(results.error.errors, 2)
        followerId = results.error.followerIdx(k);
        plot(ax, results.error.time, results.error.errors(:, k), ...
            'LineWidth', 1.5, 'DisplayName', sprintf('follower %d', followerId));
    end

    meanCurve = mean(results.error.errors, 2);
    plot(ax, results.error.time, meanCurve, 'k--', 'LineWidth', 2.0, ...
        'DisplayName', 'mean error');
    xlabel(ax, 'time step');
    ylabel(ax, 'formation error [m]');
    title(ax, 'Follower Formation Error');
    legend(ax, 'Location', 'best');

    if isfield(results, 'outputDir') && exist(results.outputDir, 'dir')
        try
            saveas(fig, fullfile(results.outputDir, 'formation_error_curve.png'));
        catch
        end
    end
end

function count = selectedObstacleCount(app)
    values = [10, 20, 30];
    idx = get(app.obstaclePopup, 'Value');
    count = values(idx);
end

function setStatus(app, lines)
    set(app.statusText, 'String', lines);
end

function lines = buildErrorLines(err)
    lines = [{'运行失败:'}; splitLines(err.message)];
    if ~isempty(err.stack)
        for i = 1:min(numel(err.stack), 4)
            frame = err.stack(i);
            lines{end + 1, 1} = sprintf('%s 第 %d 行', frame.name, frame.line); %#ok<AGROW>
        end
    end
end

function state = stateAtTime(states, t)
    idx = find(states(:, 4) >= t, 1, 'first');
    if isempty(idx)
        state = states(end, :);
        state(4) = t;
    else
        state = states(idx, :);
        state(4) = t;
    end
end

function [corners, rear, front] = vehicleGeometry(state, params)
    halfW = params.carWidth / 2;
    local = [ ...
        params.LF,  halfW; ...
        params.LF, -halfW; ...
       -params.LB, -halfW; ...
       -params.LB,  halfW];
    yaw = state(3);
    rot = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
    corners = local * rot.' + state(1:2);
    front = mean(corners(1:2, :), 1);
    rear = mean(corners(3:4, :), 1);
end

function out = splitLines(text)
    parts = regexp(text, '\n', 'split');
    out = parts(:);
end
