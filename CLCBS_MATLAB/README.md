# CLCBS_MATLAB

This directory contains a MATLAB port of the CL-CBS demo following the requested layout:

```text
CLCBS_MATLAB/
├── main.m
├── gui_main.m
├── map/{CreateMap,DefaultScenario}.m
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
- The default formation contains seven robots numbered `0` to `6`.
  Starts are `[30,25,0]`, `[24,31,0]`, `[24,19,0]`, `[18,31,0]`, `[18,19,0]`, `[12,31,0]`, `[12,19,0]`.
  Goals are `[130,25,0]`, `[124,31,0]`, `[124,19,0]`, `[118,31,0]`, `[118,19,0]`, `[112,31,0]`, `[112,19,0]`.
- `CreateMap.m` generates random circular obstacles and rejects obstacles too close to starts/goals. It also checks grid reachability so every robot has a feasible start-to-goal corridor.
- `gui_main.m` provides a dropdown to choose `10`, `20`, or `30` random obstacles.
- GUI animation updates existing vehicle graphics handles frame-by-frame instead of clearing and redrawing the axes, avoiding flicker during playback.
- After a successful GUI run, the follower formation error curve is displayed and also saved as `formation_error_curve.png`.
- The high-level planner keeps the CL-CBS conflict tree, body-conflict detection, constraint generation, and constrained low-level replanning workflow.
- The low-level planner keeps the car-like forward/reverse motion primitives, Dubins-style heuristic, and spatiotemporal constraint checks.
- The original C++ implementation uses OMPL Reeds-Shepp analytic expansion. This MATLAB version avoids external dependencies and uses an approximate Dubins-style connector near the goal.
- `main.m` saves `clcbs_results.mat`, per-agent path CSV files, formation error CSV, and `trajectory.png` under `result/save_results/`.
