# MATLAB: CL-CBS 编队验证（R2020a，多文件版本）

按你的要求，当前版本已改为**多文件结构**，并且先聚焦 **CL-CBS**，暂不包含“本文算法”和“CL-CBS-II”。

## 1. 目录结构

```text
matlab/
├── run_cl_cbs_formation_validation.m     % 兼容入口（会调用 clcbs/Main_CLCBS_Validation）
└── clcbs/
    ├── Main_CLCBS_Validation.m           % 主入口（推荐直接调用）
    ├── init_clcbs_config.m               % 参数配置
    ├── build_scenarios.m                 % 实验场景列表
    ├── generate_map_set.m                % 随机地图生成 + 可达性校验 + 地图保存
    ├── run_clcbs_case.m                  % 单地图 CL-CBS 规划
    ├── compute_clcbs_metrics.m           % 总时间/编队误差统计
    ├── draw_map_and_paths.m              % 路径绘图
    ├── plot_formation_layout.m           % 编队示意图
    ├── astar_grid_path.m                 % A* 网格路径规划
    ├── ...（冲突检测、同步、几何辅助函数）
    ├── MapData/                          % 保存每个 case 的地图 .mat
    ├── ShapeImage/                       % 保存每个 case 的地图预览图
    └── Results/                          % 保存路径图、统计表、结果 mat
```

## 2. 当前实验内容（CL-CBS）

- 地图大小：`150m x 50m`
- 机器人：7 台（0,1,2 为领航；3,4,5,6 为跟随）
- 机器人尺寸：`1m x 2m`
- 起终状态：已按你给定的数据写入配置
- 障碍物：随机生成，自动避开起点/终点附近，且校验每台机器人可达
- 默认地图数：9 张（障碍物 `10/20/30`，每组 3 张）
- 指标：
  - 编队总时间（`totalTime`）
  - 编队误差（`formationErrorSeries` 和 `meanFormationError`）

## 3. 运行方式（MATLAB 2020a）

推荐直接运行：

```matlab
addpath('matlab/clcbs');
results = Main_CLCBS_Validation();
```

兼容旧入口也可：

```matlab
results = run_cl_cbs_formation_validation();
```

## 4. 修改障碍物数量

```matlab
cfg = struct();
cfg.obstacleGroups = [10 20 30];  % 改成你需要的组
cfg.mapsPerGroup = 3;             % 每组地图数
addpath('matlab/clcbs');
results = Main_CLCBS_Validation(cfg);
```

## 5. 输出文件

默认输出目录：`matlab/clcbs/`

- `MapData/case_XX_map.mat`：随机地图数据
- `ShapeImage/case_XX_map.png`：随机地图预览图
- `Results/case_XX_clcbs.png`：每个 case 的路径/误差图
- `Results/total_time_table_clcbs.csv`：总时间统计
- `Results/formation_error_table_clcbs.csv`：编队误差统计
- `Results/clcbs_validation_results.mat`：完整实验结果
