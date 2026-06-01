function results = run_formation_planning(userConfig)
%RUN_FORMATION_PLANNING  完整三流程多机器人编队路径规划
%
% 流程1 (CL-CBS): Hybrid A* (阿克曼运动学) 初始路径 + 优先级冲突消解
% 流程2 (仿射编队): 应力矩阵 Omega + 领航者-跟随者解析路径生成
% 流程3 (轨迹优化): 安全走廊构建 + quadprog/fmincon 轨迹精化
%
% 用法:
%   results = run_formation_planning();
%   results = run_formation_planning(cfg);   % 可选自定义配置
%
% 领航者 (robots 1,2,3): 由 Hybrid A* + 极坐标偏移确定路径
% 跟随者 (robots 4,5,6,7): 由应力矩阵解析公式 p_f = -Omega_ff^{-1}*Omega_fl*p_l 确定
%
% 输出结构体包含: 三个流程的路径、应力矩阵、安全走廊、性能指标

    if nargin < 1, userConfig = struct(); end

    cfg = defaultConfig();
    cfg = mergeConfig(cfg, userConfig);

    if ~exist(cfg.outputDir, 'dir'), mkdir(cfg.outputDir); end
    rng(cfg.randomSeed);

    fprintf('===== 多机器人编队路径规划 (三流程) =====\n');
    fprintf('地图: %.0f×%.0f m | 机器人: %d | 领航者: %s | 跟随者: %s\n', ...
        cfg.mapSize(1), cfg.mapSize(2), cfg.numRobots, ...
        mat2str(cfg.leaderIdx), mat2str(cfg.followerIdx));
    fprintf('阿克曼参数: 轴距=%.1fm, 最大速度=%.1fm/s, 最大转角=%.2frad\n', ...
        cfg.wheelbase, cfg.maxSpeed, cfg.maxSteeringAngle);

    % ------------------------------------------------------------------
    % 地图生成
    % ------------------------------------------------------------------
    fprintf('\n[地图生成] 障碍物数量: %d ...\n', cfg.numObstacles);
    map = generateMap(cfg);
    fprintf('  地图生成成功 (种子=%d)。\n', map.seed);

    % ==================================================================
    % 流程1: CL-CBS + Hybrid A*
    % ==================================================================
    fprintf('\n[流程1] CL-CBS + Hybrid A* 初始路径生成...\n');
    tic;
    paths_f1 = flow1_hybridAstarMultiRobot(map, cfg);
    t1 = toc;
    fprintf('  流程1 完成 (%.2f s)\n', t1);

    % ==================================================================
    % 流程2: 仿射编队路径生成
    % ==================================================================
    fprintf('\n[流程2] 仿射编队路径生成...\n');
    tic;
    [paths_f2, Omega, formInfo] = flow2_affineFormation(map, paths_f1, cfg);
    t2 = toc;
    fprintf('  流程2 完成 (%.2f s)\n', t2);

    % ==================================================================
    % 流程3: 轨迹优化
    % ==================================================================
    fprintf('\n[流程3] 轨迹优化 (安全走廊 + 二次/非线性规划)...\n');
    tic;
    [paths_f3, corridors] = flow3_trajectoryOpt(map, paths_f2, cfg);
    t3 = toc;
    fprintf('  流程3 完成 (%.2f s)\n', t3);

    % ------------------------------------------------------------------
    % 性能指标
    % ------------------------------------------------------------------
    metrics = computeAllMetrics(paths_f1, paths_f2, paths_f3, cfg);
    printMetrics(metrics);

    % ------------------------------------------------------------------
    % 结果汇总
    % ------------------------------------------------------------------
    results = struct();
    results.config      = cfg;
    results.map         = map;
    results.Omega       = Omega;
    results.formInfo    = formInfo;
    results.paths_f1    = paths_f1;
    results.paths_f2    = paths_f2;
    results.paths_f3    = paths_f3;
    results.corridors   = corridors;
    results.metrics     = metrics;
    results.timing      = struct('flow1', t1, 'flow2', t2, 'flow3', t3);

    save(fullfile(cfg.outputDir, 'formation_planning_results.mat'), 'results');
    fprintf('\n结果已保存至 %s\n', cfg.outputDir);

    if cfg.plotFigures
        visualizeResults(results);
    end

    fprintf('\n===== 规划完成 =====\n\n');
end

%% ================================================================
%  配置
%% ================================================================

function cfg = defaultConfig()
    cfg.mapSize          = [150, 50];    % 地图尺寸 [宽, 高] m
    cfg.gridRes          = 1.0;          % 占据栅格分辨率 m
    cfg.numRobots        = 7;
    cfg.leaderIdx        = [1, 2, 3];    % 领航者索引 (1-based)
    cfg.followerIdx      = [4, 5, 6, 7]; % 跟随者索引

    % 编队名义初始位置 [x, y, theta]
    cfg.startStates = [...
        30, 25,  0;   % 领航者1
        24, 31,  0;   % 领航者2
        24, 19,  0;   % 领航者3
        18, 31,  0;   % 跟随者4
        18, 19,  0;   % 跟随者5
        12, 31,  0;   % 跟随者6
        12, 19,  0];  % 跟随者7

    cfg.goalStates = [...
        130, 25,  0;
        124, 31,  0;
        124, 19,  0;
        118, 31,  0;
        118, 19,  0;
        112, 31,  0;
        112, 19,  0];

    % 机器人物理参数
    cfg.robotSize             = [1.0, 2.0];  % [宽, 长] m
    cfg.robotRadius           = hypot(1.0/2, 2.0/2);
    cfg.robotRadiusInflated   = cfg.robotRadius + 0.35;

    % 阿克曼运动学
    cfg.wheelbase             = 2.7;    % 轴距 m
    cfg.maxSpeed              = 2.0;    % 最大速度 m/s
    cfg.maxSteeringAngle      = 0.5;    % 最大转向角 rad (~28.6°)

    % Hybrid A* 参数
    cfg.hybridDt              = 1.0;    % 时间步 s
    cfg.numSteeringAngles     = 5;      % 转向角离散数 (含0)
    cfg.hybridXRes            = 2.0;    % visited 栅格 x 分辨率 m
    cfg.hybridYRes            = 2.0;    % visited 栅格 y 分辨率 m
    cfg.hybridThRes           = pi/8;   % visited 栅格 theta 分辨率 rad
    cfg.hybridGoalTol         = 4.0;    % 目标接受半径 m
    cfg.hybridHWeight         = 1.5;    % 启发函数权重 (weighted A*)
    cfg.hybridMaxNodes        = 80000;  % 最大节点数

    % 冲突消解
    cfg.interRobotSafeDist    = 2.5;    % 最小机器人间距 m
    cfg.maxConflictIter       = 400;

    % 随机种子
    cfg.randomSeed            = 20260601;

    % 障碍物采样
    cfg.numObstacles          = 15;
    cfg.obstacleRadiusRange   = [1.5, 3.0];
    cfg.obstacleSpawnMargin   = 4.0;
    cfg.startGoalClearance    = 7.0;
    cfg.minObstacleGap        = 1.0;
    cfg.maxObsSampleAttempts  = 12000;
    cfg.maxMapAttempts        = 60;

    % 仿射编队
    cfg.nominalPositions      = [];   % 由 startStates 导出, 见下
    cfg.formationDt           = 1.0;  % 采样时间步 s

    % 安全走廊
    cfg.corridorExpandStep    = 0.5;  % 扩展步长 m
    cfg.corridorMinHW         = cfg.robotRadiusInflated + 0.3;

    % 轨迹优化
    cfg.optUseFmincon         = true;  % 是否用 fmincon (含运动学约束)
    cfg.optSmoothWeight       = 0.08;  % 平滑正则化权重
    cfg.optMaxIter            = 800;

    % 可视化
    cfg.plotFigures           = true;
    cfg.saveFigures           = true;
    cfg.outputDir = fullfile(fileparts(mfilename('fullpath')), 'results_fp');

    cfg.nominalPositions = cfg.startStates(:, 1:2);
end

function cfg = mergeConfig(base, patch)
    cfg = base;
    flds = fieldnames(patch);
    for i = 1:numel(flds)
        cfg.(flds{i}) = patch.(flds{i});
    end
end

%% ================================================================
%  地图生成
%% ================================================================

function map = generateMap(cfg)
    starts  = cfg.startStates(:, 1:2);
    goals   = cfg.goalStates(:, 1:2);
    anchors = [starts; goals];

    for attempt = 1:cfg.maxMapAttempts
        rng(cfg.randomSeed + attempt - 1);
        obs = sampleObstacles(cfg, cfg.numObstacles, anchors);
        if size(obs,1) < cfg.numObstacles, continue; end
        occ = buildOccGrid(cfg, obs);

        feasible = true;
        for rid = 1:cfg.numRobots
            [~, ok] = astar2D(starts(rid,:), goals(rid,:), occ, cfg);
            if ~ok, feasible = false; break; end
        end
        if feasible
            map.obstacles  = obs;
            map.occupancy  = occ;
            map.seed       = cfg.randomSeed + attempt - 1;
            return;
        end
    end
    error('run_formation_planning: 无法生成可通行地图，请调整参数。');
end

function obs = sampleObstacles(cfg, numObs, anchors)
    obs = zeros(numObs, 3);
    accepted = 0;
    for trial = 1:cfg.maxObsSampleAttempts
        r = cfg.obstacleRadiusRange(1) + ...
            (cfg.obstacleRadiusRange(2)-cfg.obstacleRadiusRange(1))*rand();
        x = cfg.obstacleSpawnMargin + ...
            (cfg.mapSize(1)-2*cfg.obstacleSpawnMargin)*rand();
        y = cfg.obstacleSpawnMargin + ...
            (cfg.mapSize(2)-2*cfg.obstacleSpawnMargin)*rand();

        if any(vecnorm(anchors-[x,y],2,2) < cfg.startGoalClearance+r+cfg.robotRadius)
            continue;
        end
        if accepted > 0
            ex = obs(1:accepted,:);
            if any(vecnorm(ex(:,1:2)-[x,y],2,2) < ex(:,3)+r+cfg.minObstacleGap)
                continue;
            end
        end
        accepted = accepted + 1;
        obs(accepted,:) = [x,y,r];
        if accepted == numObs, break; end
    end
    obs = obs(1:accepted,:);
end

function occ = buildOccGrid(cfg, obs)
    xs = 0:cfg.gridRes:cfg.mapSize(1);
    ys = 0:cfg.gridRes:cfg.mapSize(2);
    [X,Y] = meshgrid(xs, ys);
    occ = false(size(X));
    infl = cfg.robotRadius;
    for i = 1:size(obs,1)
        occ = occ | (hypot(X-obs(i,1), Y-obs(i,2)) <= obs(i,3)+infl);
    end
    for rid = 1:cfg.numRobots
        [sx,sy] = worldToGrid(cfg.startStates(rid,1:2), cfg);
        [gx,gy] = worldToGrid(cfg.goalStates(rid,1:2), cfg);
        occ(sy,sx) = false; occ(gy,gx) = false;
    end
end

%% ================================================================
%  流程1: CL-CBS + Hybrid A*
%% ================================================================

function paths = flow1_hybridAstarMultiRobot(map, cfg)
% 步骤1: 每台机器人独立 Hybrid A* 规划
% 步骤2: 优先级冲突消解 (CL-CBS 简化版)

    n   = cfg.numRobots;
    occ = map.occupancy;
    paths = cell(n, 1);

    fprintf('  [1/2] Hybrid A* 单机规划:\n');
    for rid = 1:n
        sState  = cfg.startStates(rid,:);   % [x, y, theta]
        goalXY  = cfg.goalStates(rid,1:2);

        fprintf('    机器人%d ... ', rid);
        [p3d, ok] = hybridAstarPlan(sState, goalXY, occ, cfg);

        if ok
            fprintf('成功 (%d 节点)\n', size(p3d,1));
            paths{rid} = p3d(:,1:2);
        else
            fprintf('Hybrid A* 失败, 回退 2D A*\n');
            [p2d, ok2] = astar2D(sState(1:2), goalXY, occ, cfg);
            if ok2
                paths{rid} = p2d;
            else
                paths{rid} = straightPathClear(sState(1:2), goalXY, map.obstacles, cfg);
            end
        end
        paths{rid} = smoothPath(paths{rid}, 2);
        paths{rid}(1,:)   = sState(1:2);
        paths{rid}(end,:) = goalXY;
    end

    fprintf('  [2/2] 时空冲突消解 ...\n');
    paths = resolveConflicts(paths, cfg);
    nConflicts = countConflicts(paths, cfg.interRobotSafeDist);
    fprintf('    剩余冲突: %d\n', nConflicts);
end

% ------------------------------------------------------------------
%  Hybrid A* (阿克曼运动学)
% ------------------------------------------------------------------
function [path, success] = hybridAstarPlan(startState, goalXY, occ, cfg)
% Hybrid A*: state = [x, y, theta], 阿克曼模型展开
% visited set 基于离散化三维格
% 返回: path (Nx3 [x,y,theta]), success (bool)

    v      = cfg.maxSpeed;
    dt     = cfg.hybridDt;
    L      = cfg.wheelbase;
    nS     = cfg.numSteeringAngles;
    dMax   = cfg.maxSteeringAngle;
    hw     = cfg.hybridHWeight;
    maxN   = cfg.hybridMaxNodes;
    goalTol = cfg.hybridGoalTol;

    mapW = cfg.mapSize(1); mapH = cfg.mapSize(2);
    gRes = cfg.gridRes;

    % 离散化参数
    xR  = cfg.hybridXRes;
    yR  = cfg.hybridYRes;
    thR = cfg.hybridThRes;
    nXG = floor(mapW/xR) + 2;
    nYG = floor(mapH/yR) + 2;
    nThG = floor(2*pi/thR) + 2;

    deltaSet = linspace(-dMax, dMax, nS);

    % 各格最优 g-cost
    gBest = inf(nXG, nYG, nThG);

    % 节点池
    nSt  = zeros(maxN, 3);   % [x, y, theta]
    nG   = zeros(maxN, 1);
    nF   = zeros(maxN, 1);
    nPar = zeros(maxN, 1, 'int32');
    nCnt = 0;

    % 开放列表 (简单堆, 以 fCost 排序)
    heapF   = zeros(maxN, 1);
    heapIdx = zeros(maxN, 1, 'int32');
    heapSz  = 0;

    % 初始节点
    nCnt = nCnt + 1;
    nSt(nCnt,:) = startState;
    nG(nCnt)    = 0;
    h0 = hw * norm(startState(1:2) - goalXY);
    nF(nCnt)    = h0;
    nPar(nCnt)  = 0;
    heapSz = heapSz + 1;
    heapF(heapSz)   = h0;
    heapIdx(heapSz) = nCnt;

    gi0 = stateToGridIdx(startState, xR, yR, thR, nXG, nYG, nThG);
    gBest(gi0(1), gi0(2), gi0(3)) = 0;

    success  = false;
    goalNode = -1;

    while heapSz > 0 && nCnt < maxN
        % Pop minimum f
        [~, mi] = min(heapF(1:heapSz));
        currIdx = heapIdx(mi);
        heapF(mi)   = heapF(heapSz);
        heapIdx(mi) = heapIdx(heapSz);
        heapSz = heapSz - 1;

        curr  = nSt(currIdx,:);
        gCurr = nG(currIdx);

        % 检查目标
        if norm(curr(1:2) - goalXY) <= goalTol
            success  = true;
            goalNode = currIdx;
            break;
        end

        % 按各转向角展开
        for a = 1:nS
            delta = deltaSet(a);

            % 阿克曼运动学 (RK2 数值积分)
            th1 = curr(3);
            dth_dt = (v/L) * tan(delta);

            x_new  = curr(1) + dt * v * cos(th1 + 0.5*dt*dth_dt);
            y_new  = curr(2) + dt * v * sin(th1 + 0.5*dt*dth_dt);
            th_new = wrapAngle(th1 + dt * dth_dt);

            % 边界检查
            if x_new < 0 || x_new > mapW || y_new < 0 || y_new > mapH
                continue;
            end

            % 碰撞检查 (沿弧线采样3点)
            hit = false;
            for frac = [0.35, 0.7, 1.0]
                xi = curr(1) + frac*(x_new-curr(1));
                yi = curr(2) + frac*(y_new-curr(2));
                gxi = min(max(round(xi/gRes)+1, 1), size(occ,2));
                gyi = min(max(round(yi/gRes)+1, 1), size(occ,1));
                if occ(gyi, gxi), hit = true; break; end
            end
            if hit, continue; end

            newState = [x_new, y_new, th_new];
            gi = stateToGridIdx(newState, xR, yR, thR, nXG, nYG, nThG);

            gNew = gCurr + v*dt;
            if gNew >= gBest(gi(1),gi(2),gi(3))
                continue;
            end
            gBest(gi(1),gi(2),gi(3)) = gNew;

            if nCnt < maxN
                nCnt = nCnt + 1;
                nSt(nCnt,:)  = newState;
                nG(nCnt)     = gNew;
                hNew         = hw * norm(newState(1:2) - goalXY);
                fNew         = gNew + hNew;
                nF(nCnt)     = fNew;
                nPar(nCnt)   = currIdx;

                heapSz = heapSz + 1;
                heapF(heapSz)   = fNew;
                heapIdx(heapSz) = nCnt;
            end
        end
    end

    if success
        buf = zeros(nCnt, 3);
        pLen = 0;
        idx  = goalNode;
        while idx > 0
            pLen = pLen + 1;
            buf(pLen,:) = nSt(idx,:);
            idx = nPar(idx);
        end
        path = flip(buf(1:pLen,:), 1);
        path(end,1:2) = goalXY;
    else
        path = [];
    end
end

function gi = stateToGridIdx(s, xR, yR, thR, nXG, nYG, nThG)
    ix  = min(max(floor(s(1)/xR)+1, 1), nXG);
    iy  = min(max(floor(s(2)/yR)+1, 1), nYG);
    thn = mod(s(3), 2*pi);
    it  = min(max(floor(thn/thR)+1, 1), nThG);
    gi  = [ix, iy, it];
end

% ------------------------------------------------------------------
%  标准 2D A* (备用)
% ------------------------------------------------------------------
function [path, success] = astar2D(startXY, goalXY, occ, cfg)
    [sx,sy] = worldToGrid(startXY, cfg);
    [gx,gy] = worldToGrid(goalXY,  cfg);
    [nR,nC] = size(occ);

    if occ(sy,sx) || occ(gy,gx)
        success = false; path = []; return;
    end

    gScore    = inf(nR,nC);
    fScore    = inf(nR,nC);
    openMask  = false(nR,nC);
    closedMsk = false(nR,nC);
    parX = zeros(nR,nC,'int32');
    parY = zeros(nR,nC,'int32');

    gScore(sy,sx) = 0;
    fScore(sy,sx) = hypot(gx-sx, gy-sy);
    openMask(sy,sx) = true;

    nb = [-1,-1; 0,-1; 1,-1; -1,0; 1,0; -1,1; 0,1; 1,1];
    mc = [sqrt(2),1,sqrt(2),1,1,sqrt(2),1,sqrt(2)] * cfg.gridRes;
    success = false;

    while any(openMask(:))
        openIdx = find(openMask);
        [~,li]  = min(fScore(openIdx));
        cur     = openIdx(li);
        [cy,cx] = ind2sub([nR,nC], cur);

        if cx==gx && cy==gy, success = true; break; end

        openMask(cy,cx)  = false;
        closedMsk(cy,cx) = true;

        for k = 1:8
            nx = cx+nb(k,1); ny = cy+nb(k,2);
            if nx<1||ny<1||nx>nC||ny>nR, continue; end
            if occ(ny,nx)||closedMsk(ny,nx), continue; end
            tg = gScore(cy,cx)+mc(k);
            if tg < gScore(ny,nx)
                parX(ny,nx) = cx; parY(ny,nx) = cy;
                gScore(ny,nx) = tg;
                fScore(ny,nx) = tg + hypot(gx-nx,gy-ny)*cfg.gridRes;
                openMask(ny,nx) = true;
            end
        end
    end

    if ~success, path = []; return; end

    cx2 = gx; cy2 = gy;
    pg  = [cx2,cy2];
    while ~(cx2==sx && cy2==sy)
        px = parX(cy2,cx2); py = parY(cy2,cx2);
        if px==0&&py==0, path=[]; return; end
        pg = [px,py; pg];
        cx2 = px; cy2 = py;
    end
    path = gridToWorld(pg, cfg);
    path(1,:) = startXY; path(end,:) = goalXY;
end

% ------------------------------------------------------------------
%  冲突消解 (优先级规划)
% ------------------------------------------------------------------
function paths = resolveConflicts(paths, cfg)
    for iter = 1:cfg.maxConflictIter
        [hasC, ri, ti] = findFirstConflict(paths, cfg.interRobotSafeDist);
        if ~hasC, return; end
        p  = paths{ri};
        ti = max(2, min(ti, size(p,1)));
        hp = p(ti-1,:);
        p  = [p(1:ti-1,:); hp; p(ti:end,:)];
        paths{ri} = p;
    end
end

function [hasC, robot, step] = findFirstConflict(paths, minDist)
    hasC = false; robot = 0; step = 0;
    synced = syncPaths(paths);
    n = numel(synced);
    T = size(synced{1},1);
    for t = 2:T
        for i = 1:n-1
            for j = i+1:n
                if norm(synced{i}(t,:)-synced{j}(t,:)) < minDist
                    hasC = true; robot = max(i,j); step = t; return;
                end
            end
        end
    end
end

function n = countConflicts(paths, minDist)
    n = 0;
    synced = syncPaths(paths);
    nR = numel(synced);
    T  = size(synced{1},1);
    for t = 1:T
        for i = 1:nR-1
            for j = i+1:nR
                if norm(synced{i}(t,:)-synced{j}(t,:)) < minDist
                    n = n+1;
                end
            end
        end
    end
end

%% ================================================================
%  流程2: 仿射编队路径生成
%% ================================================================

function [paths, Omega, info] = flow2_affineFormation(map, paths_f1, cfg)
% 2.1 计算应力矩阵 Omega
% 2.2 领航者路径 (极坐标偏移)
% 2.3 跟随者路径 (应力矩阵解析公式)
% 2.4 以 dt=1s 离散化

    n    = cfg.numRobots;
    lIdx = cfg.leaderIdx;
    fIdx = cfg.followerIdx;
    nL   = numel(lIdx);
    nF   = numel(fIdx);

    % ---- 2.1 应力矩阵 ----
    P_star = cfg.nominalPositions;   % N x 2
    Omega  = computeStressMatrix(P_star);

    omRank = rank(Omega, 1e-8);
    fprintf('  Omega: %dx%d, 秩=%d (期望%d), 条件数=%.2e\n', ...
        n, n, omRank, n-3, cond(Omega));
    fprintf('  Omega_ff 条件数: %.2e\n', cond(Omega(fIdx,fIdx)));

    % 验证 Omega * [1, P_star] ≈ 0
    resid = norm(Omega * [ones(n,1), P_star], 'fro');
    fprintf('  验证 ||Omega*[1,P_star]||_F = %.2e (应≈0)\n', resid);

    % ---- 2.2 领航者路径 (极坐标偏移) ----
    % 领航者1 路径来自 Flow1，重采样至 dt=1s
    leader1raw = paths_f1{lIdx(1)};
    totalLen = polylineLen(leader1raw);
    nSteps   = max(10, ceil(totalLen / cfg.maxSpeed) + 2);
    leader1  = resamplePath(leader1raw, nSteps);   % nSteps x 2
    theta_L1 = computeHeading(leader1);            % nSteps x 1

    % 初始极坐标偏移 d_12, d_13 (从名义位置算)
    d_12 = P_star(lIdx(2),:) - P_star(lIdx(1),:);
    d_13 = P_star(lIdx(3),:) - P_star(lIdx(1),:);

    fprintf('  领航者偏移: d_12=[%.1f,%.1f], d_13=[%.1f,%.1f]\n', ...
        d_12(1),d_12(2), d_13(1),d_13(2));

    % 检查不共线
    areaL = abs( (P_star(lIdx(2),1)-P_star(lIdx(1),1)) * ...
                 (P_star(lIdx(3),2)-P_star(lIdx(1),2)) - ...
                 (P_star(lIdx(3),1)-P_star(lIdx(1),1)) * ...
                 (P_star(lIdx(2),2)-P_star(lIdx(1),2)) );
    fprintf('  领航者三角形面积: %.2f m²\n', areaL);
    if areaL < 0.5
        warning('flow2: 领航者接近共线，仿射分解可能不稳定。');
    end

    % 公式 (4): p_Lk(t) = p_L1(t) + R(θ(t)) * d_1k
    leader2 = zeros(nSteps, 2);
    leader3 = zeros(nSteps, 2);
    for t = 1:nSteps
        R = rot2(theta_L1(t));
        leader2(t,:) = leader1(t,:) + (R * d_12')';
        leader3(t,:) = leader1(t,:) + (R * d_13')';
    end
    leader2 = clipToMap(leader2, cfg);
    leader3 = clipToMap(leader3, cfg);

    % ---- 2.3 跟随者路径 (公式 (5)) ----
    % p_f(t) = -Omega_ff^{-1} * Omega_fl * p_l(t)
    Omega_ff = Omega(fIdx, fIdx);
    Omega_fl = Omega(fIdx, lIdx);

    if rcond(Omega_ff) < 1e-10
        fprintf('  Omega_ff 病态 (rcond=%.2e)，使用 pinv\n', rcond(Omega_ff));
        invOff = pinv(Omega_ff);
    else
        invOff = Omega_ff \ eye(nF);
    end
    M_fl = -invOff * Omega_fl;   % nF x nL 变换矩阵

    followerPaths = cell(nF, 1);
    for k = 1:nF
        followerPaths{k} = zeros(nSteps, 2);
    end

    for t = 1:nSteps
        p_l = [leader1(t,:); leader2(t,:); leader3(t,:)];  % nL x 2
        p_f = M_fl * p_l;   % nF x 2
        for k = 1:nF
            followerPaths{k}(t,:) = p_f(k,:);
        end
    end
    for k = 1:nF
        followerPaths{k} = clipToMap(followerPaths{k}, cfg);
    end

    % ---- 2.4 组装路径 ----
    paths = cell(n, 1);
    paths{lIdx(1)} = leader1;
    paths{lIdx(2)} = leader2;
    paths{lIdx(3)} = leader3;
    for k = 1:nF
        paths{fIdx(k)} = followerPaths{k};
    end

    % ---- 信息存储 ----
    info.M_fl      = M_fl;
    info.d_12      = d_12;
    info.d_13      = d_13;
    info.nSteps    = nSteps;
    info.theta_L1  = theta_L1;
    info.leaderArea = areaL;
    info.omegaResid = resid;
end

% ------------------------------------------------------------------
%  应力矩阵计算
% ------------------------------------------------------------------
function Omega = computeStressMatrix(P_star)
% 利用正交投影构造平衡应力矩阵
% 给定名义位置 P_star (N×2), 满足:
%   Omega * 1     = 0
%   Omega * P_star = 0
%   rank(Omega)    = N - 3  (对于 R^2 中的仿射编队)
%
% 方法: Omega = I - Q*(Q'Q)^{-1}*Q', 其中 Q = [ones(N,1), P_star]

    N = size(P_star, 1);
    Q = [ones(N,1), P_star];   % N x 3

    % 使用 QR 分解提高数值稳定性
    [Qq, Qr] = qr(Q, 0);  % thin QR
    if abs(det(Qr)) < 1e-8
        warning('computeStressMatrix: 名义位置可能共线，结果退化。');
    end

    % 正交投影到 Q 的正交补
    Omega = eye(N) - Qq * Qq';

    % 对称化消除数值噪声
    Omega = (Omega + Omega') / 2;
    Omega(abs(Omega) < 1e-12) = 0;
end

%% ================================================================
%  流程3: 轨迹优化
%% ================================================================

function [optPaths, corridors] = flow3_trajectoryOpt(map, refPaths, cfg)
    n         = cfg.numRobots;
    optPaths  = cell(n, 1);
    corridors = cell(n, 1);

    for rid = 1:n
        fprintf('  机器人%d: ', rid);
        refP = refPaths{rid};
        if isempty(refP) || size(refP,1) < 2
            optPaths{rid} = refP; corridors{rid} = [];
            fprintf('路径为空，跳过\n');
            continue;
        end

        % 3.1 构建安全走廊
        corr = buildSafetyCorridors(refP, map.obstacles, cfg);
        corridors{rid} = corr;

        % 3.2 轨迹优化
        startXY = cfg.startStates(rid, 1:2);
        goalXY  = cfg.goalStates(rid, 1:2);

        if cfg.optUseFmincon
            fprintf('fmincon NLP ... ');
            optP = optimTrajNLP(refP, corr, startXY, goalXY, cfg);
        else
            fprintf('quadprog QP ... ');
            optP = optimTrajQP(refP, corr, startXY, goalXY, cfg);
        end

        optPaths{rid} = optP;
        fprintf('完成\n');
    end
end

% ------------------------------------------------------------------
%  安全走廊构建 (轴对齐矩形)
% ------------------------------------------------------------------
function corridors = buildSafetyCorridors(path, obstacles, cfg)
% 对路径每个采样点, 向四方向扩展直到碰到障碍物或地图边界
% 返回 N×4 矩阵 [xmin, xmax, ymin, ymax]

    N    = size(path, 1);
    corr = zeros(N, 4);
    step = cfg.corridorExpandStep;
    hw   = cfg.corridorMinHW;
    mapW = cfg.mapSize(1);
    mapH = cfg.mapSize(2);

    for k = 1:N
        cx = path(k,1); cy = path(k,2);

        xlo = max(0,    cx - hw);
        xhi = min(mapW, cx + hw);
        ylo = max(0,    cy - hw);
        yhi = min(mapH, cy + hw);

        % 向右 (+x)
        while xhi+step <= mapW
            if rectClearOfObstacles(xhi+step, ylo, yhi, 'xedge', obstacles, cfg)
                xhi = xhi + step;
            else
                break;
            end
        end
        % 向左 (-x)
        while xlo-step >= 0
            if rectClearOfObstacles(xlo-step, ylo, yhi, 'xedge', obstacles, cfg)
                xlo = xlo - step;
            else
                break;
            end
        end
        % 向上 (+y)
        while yhi+step <= mapH
            if rectClearOfObstacles(yhi+step, xlo, xhi, 'yedge', obstacles, cfg)
                yhi = yhi + step;
            else
                break;
            end
        end
        % 向下 (-y)
        while ylo-step >= 0
            if rectClearOfObstacles(ylo-step, xlo, xhi, 'yedge', obstacles, cfg)
                ylo = ylo - step;
            else
                break;
            end
        end

        corr(k,:) = [xlo, xhi, ylo, yhi];
    end
    corridors = corr;
end

function ok = rectClearOfObstacles(edgeVal, lo, hi, direction, obstacles, cfg)
% 检查矩形新扩展边缘是否无障碍
% 采样边缘上若干点
    nSample = 5;
    ok = true;
    r = cfg.robotRadius + 0.1;

    if strcmp(direction, 'xedge')
        % 扩展 x 边: 检查 (edgeVal, y) for y in [lo,hi]
        ys = linspace(lo, hi, nSample);
        for i = 1:nSample
            pt = [edgeVal, ys(i)];
            for oi = 1:size(obstacles,1)
                if norm(pt - obstacles(oi,1:2)) < obstacles(oi,3) + r
                    ok = false; return;
                end
            end
        end
    else
        % 扩展 y 边: 检查 (x, edgeVal) for x in [lo,hi]
        xs = linspace(lo, hi, nSample);
        for i = 1:nSample
            pt = [xs(i), edgeVal];
            for oi = 1:size(obstacles,1)
                if norm(pt - obstacles(oi,1:2)) < obstacles(oi,3) + r
                    ok = false; return;
                end
            end
        end
    end
end

% ------------------------------------------------------------------
%  轨迹优化: QP (位置层面, quadprog)
% ------------------------------------------------------------------
function optPath = optimTrajQP(refPath, corridors, startXY, goalXY, cfg)
% 最小化 ||p - p_ref||^2 + lambda*||平滑度||^2
% 受约束于: 走廊上下界, 起终点等式

    T     = size(refPath, 1);
    nVars = 2 * T;

    refFlat = reshape(refPath', nVars, 1);

    % 平滑度正则化: 二阶差分矩阵
    lam = cfg.optSmoothWeight;
    e   = ones(T,1);
    D2  = spdiags([e, -2*e, e], 0:2, T-2, T);  % 二阶差分
    Dreg = kron(D2, eye(2));
    H   = 2*(speye(nVars) + lam*(Dreg'*Dreg));
    fv  = -2*refFlat;

    % 走廊上下界
    lb = zeros(nVars,1); ub = zeros(nVars,1);
    for t = 1:T
        lb(2*t-1) = max(0,            corridors(t,1));
        ub(2*t-1) = min(cfg.mapSize(1), corridors(t,2));
        lb(2*t)   = max(0,            corridors(t,3));
        ub(2*t)   = min(cfg.mapSize(2), corridors(t,4));
    end

    % 起终点等式约束
    Aeq = zeros(4, nVars); beq = zeros(4,1);
    Aeq(1,1) = 1; beq(1) = startXY(1);
    Aeq(2,2) = 1; beq(2) = startXY(2);
    Aeq(3,nVars-1) = 1; beq(3) = goalXY(1);
    Aeq(4,nVars)   = 1; beq(4) = goalXY(2);

    qpOpts = optimoptions('quadprog', 'Display','none', ...
        'MaxIterations', cfg.optMaxIter);

    [xOpt, ~, flag] = quadprog(H, fv, [], [], Aeq, beq, lb, ub, refFlat, qpOpts);

    if flag > 0
        optPath = reshape(xOpt, 2, T)';
    else
        % 若 QP 不可行, 将参考路径投影到走廊内
        optPath = refPath;
        for t = 1:T
            optPath(t,1) = min(max(optPath(t,1), corridors(t,1)), corridors(t,2));
            optPath(t,2) = min(max(optPath(t,2), corridors(t,3)), corridors(t,4));
        end
    end

    optPath(1,:)   = startXY;
    optPath(end,:) = goalXY;
end

% ------------------------------------------------------------------
%  轨迹优化: NLP (含阿克曼运动学约束, fmincon)
% ------------------------------------------------------------------
function optPath = optimTrajNLP(refPath, corridors, startXY, goalXY, cfg)
% 变量: x = [x1,y1,th1,...,xT,yT,thT, v1,d1,...,v_{T-1},d_{T-1}]
% 目标: min sum_t ||(x_t,y_t)-ref_t||^2 + lambda*(转向平滑)
% 等式约束: 阿克曼运动学 (RK2) + 起点固定
% 说明: 嵌套函数 objFcn/conFcn 定义在所有可执行代码之后

    T    = size(refPath,1);
    v0   = cfg.maxSpeed;
    L    = cfg.wheelbase;
    dt   = cfg.formationDt;
    lam  = cfg.optSmoothWeight;
    nSt  = 3*T;
    nV   = nSt + 2*(T-1);

    refHd = computeHeading(refPath);

    % 初始猜测
    x0 = zeros(nV,1);
    for t = 1:T
        x0(3*t-2) = refPath(t,1);
        x0(3*t-1) = refPath(t,2);
        x0(3*t)   = refHd(t);
    end
    for t = 1:T-1
        x0(nSt+2*t-1) = v0;
        x0(nSt+2*t)   = 0;
    end

    % 变量上下界
    lb = -inf(nV,1); ub = inf(nV,1);
    for t = 1:T
        lb(3*t-2) = max(0,              corridors(t,1));
        ub(3*t-2) = min(cfg.mapSize(1), corridors(t,2));
        lb(3*t-1) = max(0,              corridors(t,3));
        ub(3*t-1) = min(cfg.mapSize(2), corridors(t,4));
    end
    for t = 1:T-1
        lb(nSt+2*t-1) = 0;          ub(nSt+2*t-1) = v0*1.5;
        lb(nSt+2*t)   = -cfg.maxSteeringAngle;
        ub(nSt+2*t)   =  cfg.maxSteeringAngle;
    end

    % 起点等式约束
    Aeq = zeros(3,nV); beq = zeros(3,1);
    Aeq(1,1) = 1; beq(1) = startXY(1);
    Aeq(2,2) = 1; beq(2) = startXY(2);
    Aeq(3,3) = 1; beq(3) = refHd(1);

    nlpOpts = optimoptions('fmincon', ...
        'Display',               'none', ...
        'Algorithm',             'interior-point', ...
        'MaxFunctionEvaluations', 30000, ...
        'MaxIterations',          cfg.optMaxIter, ...
        'ConstraintTolerance',    1e-3, ...
        'OptimalityTolerance',    1e-4);

    % 调用 fmincon — 嵌套函数在下方定义，MATLAB 允许前向引用
    try
        [xOpt, ~, flag] = fmincon(@objFcn, x0, [], [], Aeq, beq, lb, ub, @conFcn, nlpOpts);
    catch ME
        fprintf('(NLP异常:%s 回退QP)', ME.message);
        optPath = optimTrajQP(refPath, corridors, startXY, goalXY, cfg);
        return;
    end

    if flag >= 0
        optPath = zeros(T,2);
        for t = 1:T
            optPath(t,1) = xOpt(3*t-2);
            optPath(t,2) = xOpt(3*t-1);
        end
    else
        fprintf('(NLP flag=%d 回退QP)', flag);
        optPath = optimTrajQP(refPath, corridors, startXY, goalXY, cfg);
    end
    optPath(1,:)   = startXY;
    optPath(end,:) = goalXY;

    % ------------------------------------------------------------------
    % 嵌套函数 — 必须放在所有可执行代码之后、外层函数 end 之前
    % MATLAB 规则: 含嵌套函数的父函数中所有可执行语句须先于嵌套函数定义
    % ------------------------------------------------------------------

    function J = objFcn(x)
        % 位置偏差 + 转向角大小 + 转向角连续性
        J = 0;
        for tt = 1:T
            J = J + (x(3*tt-2)-refPath(tt,1))^2 + (x(3*tt-1)-refPath(tt,2))^2;
        end
        for tt = 1:T-1
            J = J + lam * x(nSt+2*tt)^2;
        end
        if T > 2
            for tt = 1:T-2
                J = J + lam * (x(nSt+2*tt) - x(nSt+2*(tt+1)))^2;
            end
        end
    end

    function [c, ceq] = conFcn(x)
        % 阿克曼运动学离散化 (RK2)
        c   = [];
        ceq = zeros(3*(T-1), 1);
        for tt = 1:T-1
            xi  = x(3*tt-2); yi  = x(3*tt-1); thi = x(3*tt);
            vt  = x(nSt+2*tt-1);
            dst = x(nSt+2*tt);
            dth  = (vt/L)*tan(dst);
            xi1p = xi + dt*vt*cos(thi + 0.5*dt*dth);
            yi1p = yi + dt*vt*sin(thi + 0.5*dt*dth);
            ceq(3*tt-2) = x(3*tt+1) - xi1p;
            ceq(3*tt-1) = x(3*tt+2) - yi1p;
            ceq(3*tt)   = x(3*tt+3) - wrapAngle(thi + dt*dth);
        end
    end

end

%% ================================================================
%  性能指标
%% ================================================================

function metrics = computeAllMetrics(p1, p2, p3, cfg)
    metrics.flow1 = singleFlowMetrics(p1, cfg);
    metrics.flow2 = singleFlowMetrics(p2, cfg);
    metrics.flow3 = singleFlowMetrics(p3, cfg);
end

function m = singleFlowMetrics(paths, cfg)
    n  = numel(paths);
    ll = zeros(n,1);
    for i = 1:n, ll(i) = polylineLen(paths{i}); end
    m.pathLengths = ll;
    m.totalTime   = max(ll) / cfg.maxSpeed;

    synced = syncPaths(paths);
    T = size(synced{1},1);
    errSer = zeros(T,1);
    ref = synced{1};
    fIdx = cfg.followerIdx;
    nomP = cfg.nominalPositions;
    for t = 1:T
        e = 0;
        for k = 1:numel(fIdx)
            rid = fIdx(k);
            desired = ref(t,:) + nomP(rid,:) - nomP(1,:);
            e = e + norm(synced{rid}(t,:) - desired);
        end
        errSer(t) = e / numel(fIdx);
    end
    m.formationErrSeries = errSer;
    m.meanFormationErr   = mean(errSer);
    m.maxFormationErr    = max(errSer);
end

function printMetrics(m)
    fprintf('\n=== 性能指标汇总 ===\n');
    fprintf('%-16s  总时间(s)  平均编队误差(m)  最大编队误差(m)\n', '');
    names = {'流程1(CL-CBS)', '流程2(仿射编队)', '流程3(优化后)'};
    flows = {m.flow1, m.flow2, m.flow3};
    for i = 1:3
        fprintf('  %-14s  %9.3f  %15.4f  %15.4f\n', ...
            names{i}, flows{i}.totalTime, flows{i}.meanFormationErr, flows{i}.maxFormationErr);
    end
    fprintf('\n');
end

%% ================================================================
%  工具函数
%% ================================================================

function R = rot2(theta)
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
end

function th = wrapAngle(theta)
    th = mod(theta + pi, 2*pi) - pi;
end

function heading = computeHeading(path)
    N = size(path,1);
    heading = zeros(N,1);
    for i = 1:N-1
        dx = path(i+1,1)-path(i,1);
        dy = path(i+1,2)-path(i,2);
        if hypot(dx,dy) > 1e-6
            heading(i) = atan2(dy,dx);
        elseif i > 1
            heading(i) = heading(i-1);
        end
    end
    heading(N) = heading(N-1);
    % Gaussian smoothing
    kernel = [0.1; 0.2; 0.4; 0.2; 0.1];
    heading = conv(heading, kernel, 'same');
end

function p = resamplePath(path, nSamples)
    if size(path,1) <= 1
        p = repmat(path(1,:), nSamples, 1); return;
    end
    seg = vecnorm(diff(path,1,1),2,2);
    s   = [0; cumsum(seg)];
    if s(end) < 1e-6
        p = repmat(path(1,:), nSamples, 1); return;
    end
    q = linspace(0, s(end), nSamples)';
    p = [interp1(s, path(:,1), q, 'linear'), ...
         interp1(s, path(:,2), q, 'linear')];
end

function path = clipToMap(path, cfg)
    path(:,1) = min(max(path(:,1), 0), cfg.mapSize(1));
    path(:,2) = min(max(path(:,2), 0), cfg.mapSize(2));
end

function len = polylineLen(path)
    if isempty(path) || size(path,1) < 2, len = 0; return; end
    len = sum(vecnorm(diff(path,1,1),2,2));
end

function p = smoothPath(path, iters)
    p = path;
    if size(path,1) < 4, return; end
    for it = 1:iters
        q = p;
        q(2:end-1,:) = 0.2*p(1:end-2,:) + 0.6*p(2:end-1,:) + 0.2*p(3:end,:);
        q(1,:) = p(1,:); q(end,:) = p(end,:);
        p = q;
    end
end

function p = straightPathClear(startXY, goalXY, obstacles, cfg)
    n = 60;
    t = linspace(0,1,n)';
    p = startXY + t .* (goalXY - startXY);
    for i = 1:size(p,1)
        for oi = 1:size(obstacles,1)
            c = obstacles(oi,1:2);
            r = obstacles(oi,3) + cfg.robotRadius + 0.3;
            v = p(i,:) - c;
            d = norm(v);
            if d < r
                if d < 1e-6, v=[1,0]; d=1; end
                p(i,:) = c + (v/d)*r;
            end
        end
        p(i,:) = min(max(p(i,:), [0,0]), cfg.mapSize);
    end
end

function synced = syncPaths(paths)
    n    = numel(paths);
    lens = cellfun(@(p) size(p,1), paths);
    maxL = max(lens);
    synced = cell(n,1);
    for i = 1:n
        p = paths{i};
        if size(p,1) < maxL
            p = [p; repmat(p(end,:), maxL-size(p,1), 1)];
        end
        synced{i} = p;
    end
end

function [gx,gy] = worldToGrid(xy, cfg)
    gx = min(max(round(xy(1)/cfg.gridRes)+1, 1), round(cfg.mapSize(1)/cfg.gridRes)+1);
    gy = min(max(round(xy(2)/cfg.gridRes)+1, 1), round(cfg.mapSize(2)/cfg.gridRes)+1);
end

function path = gridToWorld(pg, cfg)
    path = (double(pg)-1)*cfg.gridRes;
end

%% ================================================================
%  可视化
%% ================================================================

function visualizeResults(results)
    cfg = results.config;

    figH = figure('Name','编队路径规划 - 三流程结果', ...
                  'Color','w', 'Position',[40 40 1700 950]);
    tl = tiledlayout(2, 4, 'Padding','compact', 'TileSpacing','compact');
    title(tl, '多机器人编队路径规划 — 流程1 (CL-CBS) | 流程2 (仿射编队) | 流程3 (轨迹优化)', ...
        'FontSize',12,'FontWeight','bold');

    % 1. 流程1 路径图
    nexttile;
    drawMapPaths(results.map, results.paths_f1, cfg, '流程1: CL-CBS + Hybrid A*');

    % 2. 流程2 路径图
    nexttile;
    drawMapPaths(results.map, results.paths_f2, cfg, '流程2: 仿射编队路径');

    % 3. 流程3 路径图
    nexttile;
    drawMapPaths(results.map, results.paths_f3, cfg, '流程3: 优化后轨迹');

    % 4. 应力矩阵热图
    nexttile;
    imagesc(results.Omega);
    colorbar; axis equal tight;
    xlabel('机器人编号'); ylabel('机器人编号');
    set(gca,'XTick',1:cfg.numRobots,'YTick',1:cfg.numRobots, ...
        'XTickLabel',arrayfun(@num2str,1:cfg.numRobots,'uni',0), ...
        'YTickLabel',arrayfun(@num2str,1:cfg.numRobots,'uni',0));
    title('应力矩阵 \Omega');

    % 5. 编队误差曲线 (跨越两列)
    nexttile([1,2]);
    hold on;
    e1 = results.metrics.flow1.formationErrSeries;
    e2 = results.metrics.flow2.formationErrSeries;
    e3 = results.metrics.flow3.formationErrSeries;
    t1v = (0:numel(e1)-1)*cfg.formationDt;
    t2v = (0:numel(e2)-1)*cfg.formationDt;
    t3v = (0:numel(e3)-1)*cfg.formationDt;
    plot(t1v, e1, 'b-',  'LineWidth',2.0, 'DisplayName','流程1 CL-CBS');
    plot(t2v, e2, 'r--', 'LineWidth',2.0, 'DisplayName','流程2 仿射编队');
    plot(t3v, e3, 'g-',  'LineWidth',2.0, 'DisplayName','流程3 优化后');
    grid on; box on;
    xlabel('时间 (s)'); ylabel('平均编队误差 (m)');
    legend('Location','northeast');
    title('编队误差随时间变化');
    hold off;

    % 6. 安全走廊 (机器人1)
    nexttile;
    drawCorridorAndPath(results.map, results.paths_f2{1}, ...
        results.corridors{1}, results.paths_f3{1}, cfg, '安全走廊 (机器人1)');

    % 7. 编队图拓扑与变换矩阵 M_fl
    nexttile;
    drawFormationSchematic(cfg, results.Omega, results.formInfo);

    if cfg.saveFigures
        fname = fullfile(cfg.outputDir, 'formation_planning_summary.png');
        saveas(figH, fname);
        fprintf('汇总图已保存: %s\n', fname);
    end

    % 额外: 每个流程的详细路径图
    for fl = 1:3
        switch fl
            case 1, pths = results.paths_f1; ttl = '流程1 CL-CBS + Hybrid A*';
            case 2, pths = results.paths_f2; ttl = '流程2 仿射编队';
            case 3, pths = results.paths_f3; ttl = '流程3 优化后轨迹';
        end
        fDetail = figure('Name', ttl, 'Color','w', 'Position',[60+fl*20, 80, 900,400]);
        drawMapPaths(results.map, pths, cfg, ttl);
        % 标注领航/跟随
        P = cfg.startStates(:,1:2);
        cmap = lines(cfg.numRobots);
        hold on;
        for i = 1:cfg.numRobots
            tag = sprintf('R%d', i);
            if ismember(i, cfg.leaderIdx), tag=[tag,'(L)']; end
            text(P(i,1)+1, P(i,2)+0.5, tag, 'Color', cmap(i,:), ...
                'FontSize',8,'FontWeight','bold');
        end
        hold off;
        if cfg.saveFigures
            fname = fullfile(cfg.outputDir, sprintf('flow%d_paths.png', fl));
            saveas(fDetail, fname);
            close(fDetail);
        end
    end
end

function drawMapPaths(map, paths, cfg, titleStr)
    hold on; axis equal; grid on; box on;
    xlim([0 cfg.mapSize(1)]); ylim([0 cfg.mapSize(2)]);
    xlabel('x (m)'); ylabel('y (m)');
    if nargin >= 4, title(titleStr); end

    % 障碍物
    th = linspace(0,2*pi,60);
    for i = 1:size(map.obstacles,1)
        obs = map.obstacles(i,:);
        fill(obs(1)+obs(3)*cos(th), obs(2)+obs(3)*sin(th), ...
            [0.25 0.25 0.25], 'FaceAlpha',0.25, 'EdgeColor',[0.1 0.1 0.1],'LineWidth',0.5);
    end

    cmap = lines(numel(paths));
    for i = 1:numel(paths)
        p = paths{i};
        if isempty(p), continue; end
        isLeader = ismember(i, cfg.leaderIdx);
        lw = 2.2 + 0.5*isLeader;
        ls = '-';
        if ~isLeader, ls = '--'; end
        plot(p(:,1), p(:,2), ls, 'Color',cmap(i,:), 'LineWidth',lw);
        plot(p(1,1), p(1,2), 'o', 'Color',cmap(i,:), ...
            'MarkerFaceColor',cmap(i,:), 'MarkerSize',6);
        plot(p(end,1), p(end,2), 's', 'Color',cmap(i,:), ...
            'MarkerFaceColor','w', 'MarkerSize',6, 'LineWidth',1.5);
    end
    hold off;
end

function drawCorridorAndPath(map, refPath, corridors, optPath, cfg, titleStr)
    hold on; axis equal; grid on; box on;
    xlim([0 cfg.mapSize(1)]); ylim([0 cfg.mapSize(2)]);
    xlabel('x (m)'); ylabel('y (m)');
    if nargin >= 6, title(titleStr); end

    % 走廊矩形
    if ~isempty(corridors)
        for k = 1:size(corridors,1)
            c = corridors(k,:);
            w = c(2)-c(1); h = c(4)-c(3);
            if w > 0 && h > 0
                rectangle('Position',[c(1),c(3),w,h], ...
                    'EdgeColor',[0.1 0.7 0.1], ...
                    'FaceColor',[0.1 0.9 0.1 0.08], ...
                    'LineWidth',0.4);
            end
        end
    end

    % 障碍物
    th = linspace(0,2*pi,60);
    for i = 1:size(map.obstacles,1)
        obs = map.obstacles(i,:);
        fill(obs(1)+obs(3)*cos(th), obs(2)+obs(3)*sin(th), ...
            [0.2 0.2 0.2], 'FaceAlpha',0.35, 'EdgeColor',[0.1 0.1 0.1]);
    end

    % 参考路径
    if ~isempty(refPath)
        plot(refPath(:,1), refPath(:,2), 'b--', 'LineWidth',1.5, 'DisplayName','参考(仿射编队)');
    end
    % 优化后路径
    if ~isempty(optPath)
        plot(optPath(:,1), optPath(:,2), 'r-',  'LineWidth',2.2, 'DisplayName','优化后轨迹');
        plot(optPath(1,1),   optPath(1,2),   'go', 'MarkerFaceColor','g','MarkerSize',8);
        plot(optPath(end,1), optPath(end,2), 'r^', 'MarkerFaceColor','r','MarkerSize',8);
    end
    legend('Location','northwest','FontSize',8);
    hold off;
end

function drawFormationSchematic(cfg, Omega, info)
    hold on; axis equal; grid on; box on;
    P = cfg.nominalPositions;
    P0 = P - P(1,:);   % 相对坐标 (以领航者1为原点)
    n = size(P,1);

    % 画应力矩阵对应的"虚拟边" (|Omega_ij| > 阈值)
    thr = max(abs(Omega(:))) * 0.05;
    for i = 1:n
        for j = i+1:n
            if abs(Omega(i,j)) > thr
                lw = min(3.5, 1 + abs(Omega(i,j))/thr * 0.3);
                plot([P0(i,1), P0(j,1)], [P0(i,2), P0(j,2)], ...
                    'k-', 'LineWidth', lw, 'Color',[0.5 0.5 0.5 0.6]);
            end
        end
    end

    cmap = lines(n);
    for i = 1:n
        px = P0(i,1); py = P0(i,2);
        isL = ismember(i, cfg.leaderIdx);
        ms  = 14;
        if isL
            plot(px,py,'s','Color',cmap(i,:),'MarkerFaceColor',cmap(i,:),'MarkerSize',ms);
        else
            plot(px,py,'o','Color',cmap(i,:),'MarkerFaceColor',cmap(i,:),'MarkerSize',ms);
        end
        tag = sprintf('R%d',i);
        if isL, tag=[tag,'\diamondsuit']; end
        text(px+0.3,py+0.3,tag,'FontSize',8,'Color',cmap(i,:));
    end

    xlabel('\Delta x (m)'); ylabel('\Delta y (m)');
    title(sprintf('编队图 (方块=领航者,圆=跟随者)\nM_{fl}条件数=%.2e', cond(info.M_fl)));
    hold off;
end
