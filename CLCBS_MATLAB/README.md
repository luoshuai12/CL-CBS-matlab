# CLCBS_MATLAB

This directory contains a MATLAB port of the CL-CBS demo following the requested layout:

```text
CLCBS_MATLAB/
├── main.m
├── map/CreateMap.m
├── high_level/{CBS,DetectConflict,GenerateChild,ComputeCost}.m
├── low_level/{HybridAstar,DubinsPlanner,CheckConstraint}.m
├── utils/{CollisionCheck,DrawMap,DrawTrajectory,InterpolatePath}.m
└── data/scenario.mat
```

Run in MATLAB:

```matlab
cd CLCBS_MATLAB
results = main();
```

Notes:

- The high-level planner keeps the CL-CBS conflict tree, body-conflict detection, constraint generation, and constrained low-level replanning workflow.
- The low-level planner keeps the car-like forward/reverse motion primitives and spatiotemporal constraint checks.
- The original C++ implementation uses OMPL Reeds-Shepp analytic expansion. This MATLAB version avoids external dependencies and uses an approximate Dubins-style connector near the goal.
