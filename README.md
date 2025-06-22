# Efficient Hierarchical Any-Angle Path Planning on Multi-Resolution 3D Grids

This repository contains a preview of the code accompanying our RSS 2025 paper:
```
Reijgwart, V., Cadena, C., Siegwart, R., & Ott, L. (2025). Efficient Hierarchical Any-Angle Path Planning on Multi-Resolution 3D Grids. Proceedings of Robotics: Science and Systems XXI.
```

The full release will include:
- A Python API (alongside the existing C++ interface)
- Demos and usage examples
- Full documentation

## Building the Code

Build the code using Docker with:

```bash
cd rss2025_paper499_code
docker build .
```

## Loading a Map

The experiments in the paper were performed using the 10cm resolution demo maps from Wavemap's documentation for the Newer College Mine, Cloister, Math, and Park environments. These maps can be downloaded [as described here](https://ethz-asl.github.io/wavemap/pages/demos.html#quick-start:~:text=one%20of%20the-,maps%20provided%20here,-Open%20Rviz%2C%20for).

After downloading a map, convert it into a traversability map using:

```c++
#include <wavemap/core/utils/query/classified_map.h>
#include <wavemap/core/utils/sdf/quasi_euclidean_sdf_generator.h>
#include <wavemap/io/file_conversions.h>

// Load the map
wavemap::MapBase::Ptr loaded_map;
const bool success =
  wavemap::io::fileToMap("/path/to/downloaded/map.wvmp", loaded_map);
if (!success) { return; }

// Classify the obstacles and pad them by the robot radius
const FloatingPoint robot_radius = 0.35;          // meters
const FloatingPoint occupancy_threshold = -0.01;  // log-odds
QuasiEuclideanSDFGenerator esdf_generator{robot_radius,
                                          occupancy_threshold};
const auto esdf = esdf_generator.generate(*loaded_map);
const auto traversability_map = std::make_shared<ClassifiedMap>(
    *loaded_map, OccupancyClassifier{occupancy_threshold},
    esdf, robot_radius);
```

## Creating a Planner

The planners discussed in the paper can be reproduced as follows.

### A*

```c++
#include "a_star.h"

AStarConfig config{};
config.heuristic_type = PointwiseHeuristicType::kOctile;

AStar planner(config, traversability_map);
```

### Theta*

```c++
#include "theta_star.h"

ThetaStar planner(traversability_map);
```

### LazyTheta*

```c++
#include "lazy_theta_star.h"

LazyThetaStar planner(traversability_map);
```

### RRT* (0.1s)

```c++
#include "ompl_planner.h"

OmplPlannerConfig config{};
config.max_solve_time = 0.1;

OmplPlanner planner(config, traversability_map, PlannerType::kRRTStar);
```

### RRT* (1s)

```c++
#include "ompl_planner.h"

OmplPlannerConfig config{};
config.max_solve_time = 1.0;

OmplPlanner planner(config, traversability_map, PlannerType::kRRTStar);
```

### RRT* (10s)

```c++
#include "ompl_planner.h"

OmplPlannerConfig config{};
config.max_solve_time = 10.0;

OmplPlanner planner(config, traversability_map, PlannerType::kRRTStar);
```

### RRTConnect

```c++
#include "ompl_planner.h"

OmplPlannerConfig config{};
config.max_solve_time = 10.0;

OmplPlanner planner(config, traversability_map, PlannerType::kRRTConnect);
```

### Ours

```c++
#include "multi_resolution_theta_star.h"

MultiResThetaStarConfig config{};
config.refinement_epsilon = 0.01f;
config.initialization_min_height = 0;  // 10cm

MultiResThetaStar planner(config, traversability_map);
```

### Ours (Lazy)

```c++
#include "multi_resolution_lazy_theta_star.h"

MultiResLazyThetaStarConfig config{};
config.refinement_epsilon = 0.01f;
config.initialization_min_height = 0;  // 10cm

MultiResLazyThetaStar planner(config, traversability_map);
```

### Ours (Fast)

```c++
#include "multi_resolution_lazy_theta_star.h"

MultiResLazyThetaStarConfig config{};
config.refinement_epsilon = 0.01f;
config.initialization_min_height = 2;  // 40cm

MultiResLazyThetaStar planner(config, traversability_map);
```

## Computing a Path

After initializing a planner, compute a path using:

```c++
Point3D start{};  // desired start position
Point3D goal{};   // desired start position
std::vector<Point3D> path = planner.searchPath(start, goal);
```
