# MATLAB: 多机器人编队路径规划（三流程完整实现）

本目录包含两个主要 MATLAB 脚本，实现完整的多机器人编队路径规划系统。

---

## 文件说明

| 文件 | 说明 |
|---|---|
| `run_formation_planning.m` | **主程序**：完整三流程实现 |
| `run_cl_cbs_formation_validation.m` | **验证程序**：三算法对比基准测试 |

---

## `run_formation_planning.m`（主程序）

完整实现三个规划流程，7 机器人（领航者 1/2/3，跟随者 4/5/6/7）。

### 流程一：CL-CBS 初始路径生成

低层规划器：**时空 Hybrid A\***（考虑阿克曼运动学约束）

- 状态空间：`(x, y, θ)`
- 转向角离散化：`[-δmax, ..., 0, ..., +δmax]`（默认 5 档）
- 运动积分：Runge-Kutta 2 阶
- Visited 集：三维网格哈希（`xRes × yRes × θRes`）
- 回退策略：Hybrid A\* 失败时自动切换至标准 2D A\*

上层冲突消解：**优先级规划（CL-CBS 简化）**

- 逐轮检测最近时空冲突，对高编号机器人插入等待点

### 流程二：仿射编队路径生成（重点）

**2.1 求解应力矩阵 Ω**

```
P* = 名义位置矩阵 (7×2)
Q  = [ones(7,1), P*]     (7×3)
Ω  = I - Q·(Q'Q)^{-1}·Q'
```

满足：`Ω·1 = 0`，`Ω·P* = 0`，`rank(Ω) = N-3 = 4`。

通过 QR 分解提升数值稳定性（`Ω = I - Qq·Qq'`，细 QR）。

**2.2 领航者选择**：机器人 1/2/3（三点不共线，验证三角形面积 > 0）

**2.3 领航者路径（极坐标偏移，公式 4）**

```
p_L2(t) = p_L1(t) + R(θ(t)) · d_12
p_L3(t) = p_L1(t) + R(θ(t)) · d_13
```

其中 `R(θ)` 为 2D 旋转矩阵，`d_1k` 为名义偏移向量，`θ(t)` 来自领航者 1 的航向角。

**2.4 跟随者路径（公式 5，应力矩阵解析求解）**

```
p_f(t) = −Ω_ff^{-1} · Ω_fl · p_l(t)
```

跟随者位置由应力矩阵和领航者位置完全解析确定，无需独立规划。验证：初始 `||Ω·[1,P*]||_F ≈ 0`。

**2.5 离散化**：以 `Δt = 1 s` 采样整条路径，记录每个机器人位置。

### 流程三：轨迹优化

**3.1 安全走廊构建**

对参考路径每个采样点，向四方向（±x, ±y）逐步扩展至障碍物或地图边界，生成轴对齐矩形走廊序列 `[xmin, xmax, ymin, ymax]`。

**3.2 二次规划（QP，默认）**

```
min  ||p - p_ref||² + λ·||D²p||²        (位置偏差 + 平滑正则化)
s.t. corridors(t, 1) ≤ x_t ≤ corridors(t, 2)
     corridors(t, 3) ≤ y_t ≤ corridors(t, 4)
     p(1) = start,  p(T) = goal
```

使用 MATLAB `quadprog` 求解（需 Optimization Toolbox）。

**3.3 非线性规划（NLP，可选，含阿克曼运动学）**

变量：`[x₁,y₁,θ₁, ..., xT,yT,θT, v₁,δ₁, ..., v_{T-1},δ_{T-1}]`

目标函数：位置偏差 + 转向角平滑；等式约束：阿克曼离散方程（RK2）；不等式：走廊边界（通过 `lb/ub`）。

使用 `fmincon`（interior-point 算法）；NLP 失败自动回退到 QP。

---

### 运行方式

```matlab
% 默认参数 (15 个障碍物, NLP 优化)
results = run_formation_planning();

% 自定义配置
cfg.numObstacles   = 25;
cfg.optUseFmincon  = false;   % 仅使用 QP
cfg.plotFigures    = true;
cfg.randomSeed     = 42;
results = run_formation_planning(cfg);
```

### 主要输出（`results_fp/`）

| 文件 | 内容 |
|---|---|
| `formation_planning_results.mat` | 完整结果结构体 |
| `formation_planning_summary.png` | 汇总图（路径/误差/走廊/矩阵热图） |
| `flow1_paths.png` | 流程一路径图 |
| `flow2_paths.png` | 流程二路径图（仿射编队） |
| `flow3_paths.png` | 流程三路径图（优化后） |

---

## `run_cl_cbs_formation_validation.m`（三算法对比）

在多张随机地图上对比三种方法，记录总时间与编队误差。

| 方法 | 说明 |
|---|---|
| **本文算法（Proposed）** | Hybrid A\* 领航者 + 应力矩阵跟随者 + 走廊 QP 优化 |
| **CL-CBS** | 独立 A\* + 时空冲突消解 |
| **CL-CBS-II** | A\* 领航者 + 追踪控制跟随者 |

```matlab
% 默认运行 (9 张地图，障碍物数 10/20/30)
results = run_cl_cbs_formation_validation();

% 自定义
cfg.obstacleGroups = [12, 24, 36];
cfg.mapsPerGroup   = 2;
results = run_cl_cbs_formation_validation(cfg);
```

输出到 `results/`：`*.mat`、`*.csv`、对比图。

---

## 依赖

- MATLAB R2019b 或更新版本（推荐）
- **Optimization Toolbox**（`quadprog` 用于 QP；`fmincon` 用于 NLP）
- 无需其他第三方工具箱

---

## 关键算法参数（默认值）

| 参数 | 值 | 说明 |
|---|---|---|
| 地图尺寸 | 150 × 50 m | |
| 机器人数 | 7 | 3 领航 + 4 跟随 |
| 车身尺寸 | 1 × 2 m | |
| 轴距 L | 2.7 m | 阿克曼参数 |
| 最大速度 v | 2.0 m/s | |
| 最大转向角 δmax | 0.5 rad（≈28.6°）| |
| Hybrid A\* 时间步 | 1 s | 每次展开行进 2 m |
| 转向角档数 | 5 | 均匀分布在 [−δmax, +δmax] |
| Hybrid A\* 格网 | 2 m × 2 m × π/8 rad | visited 集分辨率 |
| 安全走廊扩展步长 | 0.5 m | |
| 优化平滑权重 λ | 0.08 | QP/NLP 正则化 |
