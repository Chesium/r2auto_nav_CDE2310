# g3g_frontier_exploration

> **Type:** `ament_python` package
> **Purpose:** Autonomous exploration of unknown environment using frontier-based approach

## Files

| File | Lines | Purpose |
|------|-------|---------|
| `frontier_explorer.py` | ~850 | Main exploration node (extends BasicNavigator) |
| `frontier_utils.py` | ~676 | Pure-Python grid/frontier math utilities |
| `config/frontier_exploration.yaml` | | Default parameter values |
| `launch/frontier_exploration.launch.py` | | Launch with parameters |

## What is Frontier Exploration?

A **frontier** is the boundary between explored (free) space and unexplored (unknown) space on the occupancy grid map.

```
┌─────────────────────┐
│ ? ? ? ? ? ? ? ? ? ? │  ? = unknown (-1)
│ ? ? ? ? ? ? ? ? ? ? │  . = free (0-49)
│ ? ? F F F ? ? ? ? ? │  # = occupied (50+)
│ ? ? F . . . . ? ? ? │  F = frontier (free cell next to unknown)
│ ? ? F . . . . . ? ? │
│ ? ? ? . . R . . # ? │  R = robot
│ ? ? ? . . . . . # ? │
│ ? ? ? ? # # # # # ? │
│ ? ? ? ? ? ? ? ? ? ? │
└─────────────────────┘
```

The robot drives to frontiers to reveal unknown space, progressively building a complete map.

## How `frontier_explorer.py` Works

### Architecture
Extends `BasicNavigator` from `nav2_simple_commander` - inherits `goToPose()`, `getPath()`, `cancelTask()`, etc.

### Lifecycle

1. **Wait for readiness**: map received, TF available, Nav2 servers up
2. **Planning tick** (1Hz): Find frontiers -> rank -> pick best -> send Nav2 goal
3. **Progress tick** (2Hz): Monitor active goal - timeout, success, failure
4. **Completion**: No valid frontiers for 5 cycles, or encapsulation detected

### Tick-Based Architecture (Not Callback-Driven)
```python
while rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.1)
    node.tick()  # runs planning + progress checks
```
This cooperative model gives explicit control over timing.

### Frontier Quality Filtering

Each candidate frontier cell must pass ALL of:

| Check | Parameter | Default | Meaning |
|-------|-----------|---------|---------|
| Free neighbors | `min_frontier_free_neighbors` | 3 | Must have enough open space around it |
| Occupied ratio | `max_frontier_occupied_ratio` | 0.45 | Not too close to walls |
| Reachable unknown | `min_reachable_unknown_cells` | 3 | Must lead to actual unexplored area |
| Unknown span | `min_frontier_unknown_span` | 2 | Unknown region has enough width |
| Clearance | `candidate_clearance_radius` | 1 | Robot can physically fit there |

### Cluster Blacklisting

Failed navigation goals get blacklisted with a TTL:
- Invalid goal (can't plan path): 90s blacklist
- Navigation failure: 180s blacklist
- After 3 failures: permanently exhausted

### Encapsulation Detection

Smart completion check: if the robot's reachable free space is entirely enclosed by walls (no connection to map border, only tiny unknown holes), exploration is complete even if some unknown cells remain. Prevents the robot from endlessly trying to reach unreachable areas.

## frontier_utils.py - The Math Engine

### Key Data Structures

```python
@dataclass(frozen=True)
class GridMeta:
    width: int       # grid columns
    height: int      # grid rows
    resolution: float # meters per cell
    origin_x: float  # world X of cell (0,0)
    origin_y: float  # world Y of cell (0,0)

@dataclass(frozen=True)
class FrontierCluster:
    cells: tuple[int, ...]     # cell indices
    goal_cell: int             # representative cell
    goal_xy: tuple[float, float]  # world coordinates
    size: int                  # number of cells
```

### Core Algorithms

**Frontier Detection:**
1. Iterate all cells in occupancy grid
2. A cell is a frontier if: it's free AND has at least one unknown neighbor (4-connected)

**Clustering:**
1. BFS flood-fill on frontier cells using 8-connectivity
2. Each connected component is one cluster

**Goal Selection:**
1. Find cluster centroid (average row/col)
2. Snap to nearest actual cluster cell
3. If too close to robot, pick farthest eligible cell

**Information Gain:**
Count unknown cells within a radius of the goal before and after navigation. If unknown count didn't decrease enough, the goal was unproductive -> blacklist it.

## Parameters (Full List)

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `map_topic` | `/map` | Occupancy grid source |
| `occupied_threshold` | 50 | Cell value above this = obstacle |
| `frontier_min_cluster_size` | 10 | Minimum frontier cluster size |
| `min_goal_distance` | 0.35m | Don't navigate to goals this close |
| `planning_rate_hz` | 1.0 | How often to replan |
| `progress_rate_hz` | 2.0 | How often to check goal progress |
| `goal_timeout_sec` | 120.0 | Cancel goal after this long |
| `completion_patience_cycles` | 5 | Cycles with no frontiers before declaring complete |
| `autostart` | false | Start exploring immediately |

---

**See also:** [[Frontier Exploration Explained]], [[Architecture Overview]]
