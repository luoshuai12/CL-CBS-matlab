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

- The high-level planner keeps the CL-CBS conflict tree, body-conflict detection, constraint generation, and constrained low-level replanning workflow.
- The low-level planner keeps the car-like forward/reverse motion primitives, Dubins-style heuristic, and spatiotemporal constraint checks.
- The original C++ implementation uses OMPL Reeds-Shepp analytic expansion. This MATLAB version avoids external dependencies and uses an approximate Dubins-style connector near the goal.
- `main.m` saves `clcbs_results.mat`, per-agent path CSV files, formation error CSV, and `trajectory.png` under `result/save_results/`.
