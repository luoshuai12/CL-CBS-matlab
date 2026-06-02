function gui_main()
%GUI_MAIN MATLAB GUI for configuring and visualizing the CL-CBS demo.

    rootDir = fileparts(mfilename('fullpath'));
    addpath(fullfile(rootDir, 'map'));
    addpath(fullfile(rootDir, 'high_level'));
    addpath(fullfile(rootDir, 'low_level'));
    addpath(fullfile(rootDir, 'utils'));

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

    uicontrol(panel, 'Style', 'text', 'String', '最大高层迭代', ...
        'HorizontalAlignment', 'left', 'Units', 'normalized', ...
        'Position', [0.06, 0.88, 0.42, 0.05]);
    app.maxHighLevelEdit = uicontrol(panel, 'Style', 'edit', 'String', '80', ...
        'Units', 'normalized', 'Position', [0.52, 0.88, 0.38, 0.055]);

    uicontrol(panel, 'Style', 'text', 'String', '最大低层节点', ...
        'HorizontalAlignment', 'left', 'Units', 'normalized', ...
        'Position', [0.06, 0.80, 0.42, 0.05]);
    app.maxLowLevelEdit = uicontrol(panel, 'Style', 'edit', 'String', '45000', ...
        'Units', 'normalized', 'Position', [0.52, 0.80, 0.38, 0.055]);

    uicontrol(panel, 'Style', 'text', 'String', '目标容差(m)', ...
        'HorizontalAlignment', 'left', 'Units', 'normalized', ...
        'Position', [0.06, 0.72, 0.42, 0.05]);
    app.goalToleranceEdit = uicontrol(panel, 'Style', 'edit', 'String', '2.5', ...
        'Units', 'normalized', 'Position', [0.52, 0.72, 0.38, 0.055]);

    uicontrol(panel, 'Style', 'text', 'String', '动画步长(s)', ...
        'HorizontalAlignment', 'left', 'Units', 'normalized', ...
        'Position', [0.06, 0.64, 0.42, 0.05]);
    app.pauseEdit = uicontrol(panel, 'Style', 'edit', 'String', '0.05', ...
        'Units', 'normalized', 'Position', [0.52, 0.64, 0.38, 0.055]);

    app.runButton = uicontrol(panel, 'Style', 'pushbutton', 'String', '运行仿真', ...
        'Units', 'normalized', 'Position', [0.08, 0.53, 0.36, 0.07], ...
        'Callback', @runSimulation);
    app.playButton = uicontrol(panel, 'Style', 'pushbutton', 'String', '动画播放', ...
        'Units', 'normalized', 'Position', [0.54, 0.53, 0.36, 0.07], ...
        'Callback', @playAnimation);
    app.clearButton = uicontrol(panel, 'Style', 'pushbutton', 'String', '清空显示', ...
        'Units', 'normalized', 'Position', [0.08, 0.44, 0.82, 0.06], ...
        'Callback', @clearDisplay);

    app.statusText = uicontrol(panel, 'Style', 'listbox', 'String', {'等待运行...'}, ...
        'Units', 'normalized', 'Position', [0.06, 0.05, 0.88, 0.35], ...
        'Max', 2, 'Min', 0);

    guidata(fig, app);
    drawInitialMap(fig);
end

function runSimulation(src, ~)
    fig = ancestor(src, 'figure');
    app = guidata(fig);
    setStatus(app, {'运行中，请稍候...'});
    drawnow;

    cfg = struct();
    cfg.showFigure = false;
    cfg.saveResults = true;
    cfg.params = struct();
    cfg.params.maxHighLevelIterations = readScalar(app.maxHighLevelEdit, 80);
    cfg.params.maxLowLevelNodes = readScalar(app.maxLowLevelEdit, 45000);
    cfg.params.goalTolerance = readScalar(app.goalToleranceEdit, 2.5);

    try
        app.results = main(cfg);
        guidata(fig, app);
        cla(app.ax);
        axes(app.ax); %#ok<LAXES>
        DrawMap(app.results.map, app.results.starts, app.results.goals, app.results.params);
        if app.results.success
            DrawTrajectory(app.results.solution, app.results.params);
        end
        setStatus(app, buildStatusLines(app.results));
    catch err
        setStatus(app, [{'运行失败:'}; splitLines(err.message)]);
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

    for t = 0:maxT
        cla(app.ax);
        axes(app.ax); %#ok<LAXES>
        DrawMap(results.map, results.starts, results.goals, results.params);
        colors = lines(numel(results.solution));
        for i = 1:numel(results.solution)
            state = stateAtTime(results.solution{i}.states, t);
            plot(app.ax, results.solution{i}.states(:, 1), results.solution{i}.states(:, 2), ...
                ':', 'Color', colors(i, :));
            DrawVehicle(state, results.params, colors(i, :), 0.45, app.ax);
        end
        title(app.ax, sprintf('CL-CBS animation, t = %d', t));
        drawnow;
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

function drawInitialMap(fig)
    app = guidata(fig);
    [mapData, starts, goals, params] = CreateMap();
    axes(app.ax); %#ok<LAXES>
    DrawMap(mapData, starts, goals, params);
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

function setStatus(app, lines)
    set(app.statusText, 'String', lines);
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

function out = splitLines(text)
    parts = regexp(text, '\n', 'split');
    out = parts(:);
end
