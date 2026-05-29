function results = Main_CLCBS_Validation(userConfig)
% Main_CLCBS_Validation
% MATLAB R2020a-compatible CL-CBS formation validation entry.
% This version focuses only on CL-CBS.
%
% Usage:
%   results = Main_CLCBS_Validation();
%   cfg = struct('obstacleGroups', [10 20 30], 'mapsPerGroup', 3);
%   results = Main_CLCBS_Validation(cfg);

    if nargin < 1
        userConfig = struct();
    end

    cfg = init_clcbs_config();
    cfg = merge_struct_fields(cfg, userConfig);
    prepare_output_dirs(cfg);

    rng(cfg.randomSeed);

    scenarios = build_scenarios(cfg);
    mapSet = generate_map_set(cfg, scenarios);

    nCases = numel(mapSet);
    caseResults = repmat(struct(), nCases, 1);

    fprintf('== CL-CBS 实验开始 ==\n');
    fprintf('地图大小: %.1f x %.1f m\n', cfg.mapSize(1), cfg.mapSize(2));
    fprintf('障碍物组: %s, 每组地图数: %d\n', mat2str(cfg.obstacleGroups), cfg.mapsPerGroup);

    for i = 1:nCases
        fprintf('[Case %d/%d] obstacleCount=%d, seed=%d\n', ...
            i, nCases, mapSet(i).obstacleCount, mapSet(i).seed);

        runData = run_clcbs_case(cfg, mapSet(i));
        metric = compute_clcbs_metrics(cfg, runData);

        caseResults(i).caseId = i;
        caseResults(i).obstacleCount = mapSet(i).obstacleCount;
        caseResults(i).seed = mapSet(i).seed;
        caseResults(i).map = mapSet(i);
        caseResults(i).run = runData;
        caseResults(i).metric = metric;

        fprintf('  totalTime=%.3f s, meanFormationError=%.3f m\n', ...
            metric.totalTime, metric.meanFormationError);

        if cfg.saveCaseFigure
            fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1200 560]);
            subplot(1, 2, 1);
            draw_map_and_paths(mapSet(i), runData.paths, cfg);
            title(sprintf('Case %d 路径结果', i));
            subplot(1, 2, 2);
            plot(metric.formationErrorSeries, 'LineWidth', 1.8);
            grid on;
            xlabel('时间步');
            ylabel('编队误差 (m)');
            title(sprintf('Case %d 编队误差', i));

            saveas(fig, fullfile(cfg.resultDir, sprintf('case_%02d_clcbs.png', i)));
            close(fig);
        end
    end

    [totalTimeTable, formationErrorTable] = build_summary_tables(caseResults);

    results = struct();
    results.config = cfg;
    results.maps = mapSet;
    results.cases = caseResults;
    results.totalTimeTable = totalTimeTable;
    results.formationErrorTable = formationErrorTable;

    if cfg.saveTables
        writetable(totalTimeTable, fullfile(cfg.resultDir, 'total_time_table_clcbs.csv'));
        writetable(formationErrorTable, fullfile(cfg.resultDir, 'formation_error_table_clcbs.csv'));
    end

    if cfg.saveSummaryFigure
        plot_formation_layout(cfg);
        save_clcbs_summary_plot(cfg, caseResults);
    end

    save(fullfile(cfg.resultDir, 'clcbs_validation_results.mat'), 'results');

    fprintf('== CL-CBS 实验完成 ==\n');
    disp(totalTimeTable);
    disp(formationErrorTable);
end

function prepare_output_dirs(cfg)
    make_if_needed(cfg.mapDataDir);
    make_if_needed(cfg.mapImageDir);
    make_if_needed(cfg.resultDir);
end

function make_if_needed(dirPath)
    if ~exist(dirPath, 'dir')
        mkdir(dirPath);
    end
end

function [totalTimeTable, formationErrorTable] = build_summary_tables(caseResults)
    n = numel(caseResults);
    caseId = zeros(n, 1);
    obstacleCount = zeros(n, 1);
    totalTime = zeros(n, 1);
    meanFormationError = zeros(n, 1);

    for i = 1:n
        caseId(i) = caseResults(i).caseId;
        obstacleCount(i) = caseResults(i).obstacleCount;
        totalTime(i) = caseResults(i).metric.totalTime;
        meanFormationError(i) = caseResults(i).metric.meanFormationError;
    end

    totalTimeTable = table(caseId, obstacleCount, totalTime, ...
        'VariableNames', {'CaseID', 'ObstacleCount', 'CL_CBS_TotalTime'});

    formationErrorTable = table(caseId, obstacleCount, meanFormationError, ...
        'VariableNames', {'CaseID', 'ObstacleCount', 'CL_CBS_FormationError'});
end

function save_clcbs_summary_plot(cfg, caseResults)
    obsGroups = cfg.obstacleGroups(:);
    nGroup = numel(obsGroups);
    meanTime = zeros(nGroup, 1);
    meanErr = zeros(nGroup, 1);

    for g = 1:nGroup
        mask = false(numel(caseResults), 1);
        for i = 1:numel(caseResults)
            mask(i) = caseResults(i).obstacleCount == obsGroups(g);
        end
        picked = caseResults(mask);
        t = zeros(numel(picked), 1);
        e = zeros(numel(picked), 1);
        for j = 1:numel(picked)
            t(j) = picked(j).metric.totalTime;
            e(j) = picked(j).metric.meanFormationError;
        end
        meanTime(g) = mean(t);
        meanErr(g) = mean(e);
    end

    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1000 420]);
    subplot(1, 2, 1);
    bar(obsGroups, meanTime);
    grid on;
    xlabel('障碍物数量');
    ylabel('平均总时间 (s)');
    title('CL-CBS 总时间');

    subplot(1, 2, 2);
    bar(obsGroups, meanErr);
    grid on;
    xlabel('障碍物数量');
    ylabel('平均编队误差 (m)');
    title('CL-CBS 编队误差');

    saveas(fig, fullfile(cfg.resultDir, 'summary_clcbs.png'));
    close(fig);
end
