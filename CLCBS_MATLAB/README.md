# CLCBS_MATLAB

This directory contains a MATLAB port of the CL-CBS demo following the requested layout:

```text
CLCBS_MATLAB/
├── main.m
├── gui_main.m
├── map/CreateMap.m
├── high_level/{CBS,DetectConflict,GenerateChild,ComputeCost}.m
├── low_level/{HybridAstar,DubinsHeuristic,DubinsPlanner,CheckConstraint}.m
├── utils/{CollisionCheck,InterpolatePath,DrawMap,DrawVehicle,DrawTrajectory,ComputeError}.m
├── data/{map1,scenario1}.mat
└── result/save_results/
```

Run in MATLAB:

```matlab
cd CLCBS_MATLAB
results = main();
```

Run the GUI:

```matlab
cd CLCBS_MATLAB
gui_main();
```

Notes:

- The default map is `150m x 50m`; each robot footprint is `1m x 2m`.
- `CreateMap.m` generates random circular obstacles and rejects obstacles too close to starts/goals. It also checks grid reachability so every robot has a feasible start-to-goal corridor.
- `gui_main.m` provides a dropdown to choose `10`, `20`, or `30` random obstacles.
- GUI animation updates existing vehicle graphics handles frame-by-frame instead of clearing and redrawing the axes, avoiding flicker during playback.
- The high-level planner keeps the CL-CBS conflict tree, body-conflict detection, constraint generation, and constrained low-level replanning workflow.
- The low-level planner keeps the car-like forward/reverse motion primitives, Dubins-style heuristic, and spatiotemporal constraint checks.
- The original C++ implementation uses OMPL Reeds-Shepp analytic expansion. This MATLAB version avoids external dependencies and uses an approximate Dubins-style connector near the goal.
- `main.m` saves `clcbs_results.mat`, per-agent path CSV files, formation error CSV, and `trajectory.png` under `result/save_results/`.
