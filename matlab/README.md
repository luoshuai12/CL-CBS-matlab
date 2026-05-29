# MATLAB: CL-CBS 编队验证实验

本目录提供一个可直接运行的 MATLAB 脚本，用于复现你描述的 7 机器人编队验证流程：

- 总机器人数：7
- 领航者：编号 `0,1,2`
- 跟随者：编号 `3,4,5,6`
- 机器人尺寸：`1m x 2m`
- 地图大小：`150m x 50m`
- 障碍物：随机生成，并自动规避起点/终点附近区域，保证可达
- 地图数量：默认 9 张，分三组障碍物数量 `10/20/30`，每组 3 张
- 对比算法：
  1. 本文算法（Proposed）
  2. CL-CBS
  3. CL-CBS-II（领航者 CL-CBS，跟随者由编队控制律生成）

---

## 1. 运行方式

在 MATLAB 中进入本目录后执行：

```matlab
results = run_cl_cbs_formation_validation();
```

脚本会自动：

1. 生成 9 张可通行随机地图；
2. 在每张地图上运行 3 种算法；
3. 统计：
   - 编队总时间（`totalTime`）
   - 编队误差（`meanFormationError` 与误差曲线）
4. 输出图表与 csv 到 `matlab/results/`。

---

## 2. 修改障碍物数量

可通过配置结构体修改障碍物组别和每组地图数。例如：

```matlab
cfg = struct();
cfg.obstacleGroups = [12 24 36];
cfg.mapsPerGroup = 2;
results = run_cl_cbs_formation_validation(cfg);
```

---

## 3. 主要输出文件

运行后默认输出到 `matlab/results/`：

- `cl_cbs_formation_validation.mat`：全部结果数据
- `total_time_table.csv`：总时间统计表
- `formation_error_table.csv`：编队误差统计表
- `formation_layout.png`：初始编队图
- `summary_metrics.png`：分组指标柱状图
- `case_XX_comparison.png`：每张地图的路径与误差对比图

---

## 4. 说明

该 MATLAB 实现用于实验验证流程复现和指标对比，便于在论文/报告中快速开展“可调障碍物数量 + 三算法对比 + 编队误差评价”的统一仿真。
