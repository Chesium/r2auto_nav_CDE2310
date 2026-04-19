# Autonomous Exploration and Navigation Subsystem

[Home](../README.md)

## Overview

The autonomous exploration and navigation stack is organised as a layered ROS 2 system rather than as a single monolithic controller. Cartographer provides online 2D SLAM and continuously publishes the occupancy grid and TF transforms required for navigation. Nav2 provides global planning, local trajectory control, costmaps, and recovery behaviour. Above these layers, the custom `g3g_frontier_exploration` package selects exploration goals and supervises exploration progress.

This separation of responsibilities was deliberate. The exploration node does not attempt to perform low-level path planning or motion control. Instead, it focuses on selecting useful goals from the live occupancy grid, while Nav2 determines whether those goals are reachable and how the robot should move toward them safely. Compared with a simpler nearest-frontier implementation, this architecture is easier to tune, easier to debug, and more robust when goals become stale, blocked, or low value.

At system bring-up, the `g3nav2` package launches Cartographer, Nav2, RViz, and the frontier exploration stack from a single top-level launch file. This satisfies the project objective of unified ROS 2 deployment while preserving internal modularity.

## Software Architecture

The subsystem is composed of the following main components:

| Component | Package / Node | Primary Responsibility | Key Interfaces |
| --- | --- | --- | --- |
| SLAM | `cartographer_node`, `cartographer_occupancy_grid_node` | Build and update the 2D occupancy map during exploration | `/map`, `map -> odom -> base_link` TF |
| Navigation | Nav2 stack in `g3nav2` | Compute global paths, generate local trajectories, maintain costmaps, and execute recovery behaviours | `NavigateToPose`, `ComputePathToPose`, costmaps |
| Frontier exploration | `frontier_explorer` in `g3g_frontier_exploration` | Detect frontiers, rank exploration candidates, dispatch goals, and determine completion | `/map`, TF, `/exploration/current_goal`, `/exploration_complete`, `/exploration/set_enabled` |
| Post-exploration coverage | `post_exploration_traverser` in `g3g_frontier_exploration` | Continue traversing useful reachable free space after frontier exhaustion | `/post_exploration/current_goal`, `/exploration_complete`, `/exploration/set_enabled` |

This modular design keeps the mission controller focused on mission sequencing while the exploration package exposes clear interfaces for enabling exploration, detecting completion, and handing control back when map conditions change.

![nav2stack](assets/g2-report/nav2stack.png)

<p align="center">Fig: Exploration and navigation architecture</p><br>

## Navigation Stack Design

For autonomous movement within the maze, the system uses the Nav2 stack configured in the `g3nav2` package. The global planner is `nav2_navfn_planner::NavfnPlanner`, while local trajectory tracking is handled by the MPPI controller. Both the global and local costmaps use laser-based obstacle marking and inflation layers to maintain clearance from walls and dynamic obstacles.

This arrangement separates deliberation from execution:

- Cartographer maintains the live map.
- NavFn computes feasible global routes through known free space.
- MPPI tracks those routes while reacting to local obstacle costs.
- The frontier explorer decides where the robot should go next, but it does not directly command wheel motion.

Frontier exploration is therefore a goal-generation layer above Nav2 rather than a replacement for it. Once a frontier goal is selected, Nav2 is responsible for validating and executing the motion.

## Frontier Goal Selection

### Design Rationale

A naive frontier strategy would select the nearest cell on the known-unknown boundary and dispatch it immediately. In practice, that approach often produces poor goals near wall leaks, narrow corners, unreachable pockets, or regions that no longer yield useful map growth. The implemented approach therefore treats frontier selection as a filtering and ranking problem rather than a one-step nearest-neighbour lookup.

### Frontier Extraction Pipeline

Once the map, TF transform, and Nav2 action servers are available, the `frontier_explorer` planning loop performs the following steps:

1. Read the current occupancy grid and robot pose in the `map` frame.
2. Detect raw frontier cells, defined as free cells that border unknown cells.
3. Evaluate each candidate frontier using several local quality metrics.
4. Cluster valid frontier cells into connected regions.
5. Select a representative goal cell for each cluster.
6. Ask Nav2 to compute a path to each candidate goal.
7. Rank the valid candidates and dispatch the best one.

### Frontier Cell Quality Metrics

Each frontier cell is scored using local metrics before it is promoted to a candidate goal:

| Metric | Purpose | Benefit |
| --- | --- | --- |
| Minimum free-neighbour count | Ensures the candidate lies within traversable space | Rejects thin or noisy frontier fragments |
| Maximum occupied ratio in a local window | Rejects cells surrounded too heavily by obstacles | Reduces goals in tight corners or wall-adjacent leaks |
| Reachable unknown border count | Measures how much unknown space is connected to reachable free space nearby | Favors frontiers likely to expand the map |
| Unknown span estimate | Estimates whether the frontier opens into a meaningful unexplored region | Rejects single-cell leaks and tiny cracks |
| Clearance to occupied cells | Requires a minimum obstacle clearance around the candidate | Improves safety and Nav2 success rate |

These metrics make the detector substantially more selective than a simple frontier boundary scan and reduce the likelihood of repeatedly targeting unproductive regions.

### Cluster-Based Goal Selection

After filtering, the remaining frontier cells are grouped into connected clusters. A representative cell is then chosen for each cluster, with preference given to the cell nearest the cluster centroid. If that representative is too close to the robot or fails validation, the node falls back to another valid cell in the same cluster.

This cluster-based strategy provides two benefits. First, it prevents the robot from treating one continuous frontier band as many unrelated goals. Second, it enables cluster-level blacklisting and retry control, allowing the system to move on from persistently poor regions rather than oscillating among neighbouring cells.

### Goal Ranking and Dispatch

For each surviving cluster, the node requests a path from Nav2. Clusters without a valid path are rejected immediately. The remaining candidates are ranked primarily by path length, with cluster size used as a secondary preference. The shortest feasible candidate is then dispatched as the active exploration goal.

This means the method is better described as path-validated frontier ranking than as a pure nearest-frontier policy: the final choice depends not only on geometric proximity, but also on actual navigability through the current map.

## Recovery and Completion Logic

The exploration node does not assume that every issued goal is useful. Instead, it monitors failure, timeout, and low-value outcomes explicitly.

### Blacklisting and Retry Control

The node maintains temporary blacklists for frontier clusters that fail or become invalid. If Nav2 rejects a goal, if no path exists, or if the robot times out before reaching the target, that cluster is blacklisted for a configurable time-to-live. Repeated failures are also counted, and clusters can be marked permanently exhausted after a configurable fail limit.

This prevents repeated attempts toward the same problematic region and improves stability over long runs.

### Information Gain Check

Reaching a frontier does not necessarily mean the goal was useful. A target may still be poor if the surrounding unknown space does not decrease meaningfully after arrival. To address this, the node measures local unknown occupancy before and after navigation. If the reduction is too small, the cluster is blacklisted even though Nav2 technically completed the goal.

This mechanism is especially helpful in real maps that contain sensor occlusion, grazing views, and SLAM noise.

### Encapsulation-Based Completion

A second challenge is determining when exploration is truly complete. A naive implementation would stop only when no frontiers remain. In practice, SLAM maps often contain tiny unknown pockets that are fully enclosed by occupied cells and are not reachable from the robot's connected free-space region.

The node therefore performs an encapsulation check on the reachable free-space component. If that component is enclosed and borders only a small number of tiny unknown holes, the node treats exploration as complete after several confirmation cycles. This prevents endless attempts to chase map artefacts that do not correspond to meaningful unexplored territory.

## Post-Exploration Traversal

In this system, exploration completion does not necessarily mean that the robot stops immediately. After the frontier explorer publishes `/exploration_complete`, a second node, `post_exploration_traverser`, becomes active. This node samples reachable free-space viewpoints and ranks them according to expected observation value, path cost, and clearance.

The purpose of this stage is to improve residual map coverage and continue moving through meaningful reachable space even after the primary frontier set has been exhausted. This is useful when frontier-based exploration has already covered all major openings but additional viewpoints may still improve map quality.

An additional robustness feature is frontier reactivation. If the post-exploration traverser detects that meaningful frontiers have reopened, for example after a new map update reveals fresh unknown boundaries, control is handed back to the frontier explorer through `/exploration/set_enabled`. This makes the overall architecture adaptive rather than strictly one-way.

![navexploreflow](assets/g2-report/navexploreflow.png)

<p align="center">Fig: Exploration flow</p><br>

## Tuning and Parameterisation

Both Nav2 and the frontier exploration nodes are heavily parameterised so that behaviour can be adjusted without modifying source code.

### Nav2 Parameters

The Nav2 configuration file defines the following important behaviours:

- Global and local costmap resolution
- Robot radius and inflation radius
- Obstacle-layer sensor ranges
- Global planner choice and tolerances
- MPPI controller limits, critics, and trajectory scoring

These values determine the robot's safety margin, path smoothness, and ability to move through narrow spaces reliably.

### Frontier Exploration Parameters

The exploration configuration file defines the following important behaviours:

- Minimum frontier cluster size
- Minimum goal distance from the robot
- Frontier free-neighbour threshold
- Maximum occupied ratio near a frontier
- Reachable unknown-cell threshold
- Minimum frontier unknown span
- Candidate clearance radius
- Goal timeout duration
- Blacklist TTLs and cluster failure limit
- Information-gain radius and minimum information-gain threshold
- Encapsulation confirmation thresholds

Together, these parameters control the trade-off between aggressive exploration and conservative, reliable goal selection.

The table below summarises the most important tunable parameters in the current navigation and exploration stack.

| Parameter | Meaning | Current value | Effect of increasing the value |
| --- | --- | --- | --- |
| `frontier_min_cluster_size` | Minimum number of frontier cells required for a cluster to be considered meaningful | `8` | Rejects more small frontier fragments, reducing noise but potentially missing narrow openings |
| `min_goal_distance` | Minimum allowed distance between the robot and a selected frontier goal | `0.8 m` | Pushes goals farther away, reducing trivial nearby targets but making short local refinements less likely |
| `max_frontier_occupied_ratio` | Maximum allowed obstacle density around a frontier candidate within the local window | `0.45` | Accepts more cluttered regions, increasing aggressiveness but also the risk of poor wall-adjacent goals |
| `candidate_clearance_radius` | Minimum local clearance required around a frontier candidate | `1 cell` | Enforces safer obstacle stand-off, but may reject valid frontiers in tight spaces |
| `goal_timeout_sec` | Maximum time allowed for a frontier goal before cancellation and blacklisting | `15.0 s` | Makes the explorer more patient with difficult paths, but slows recovery from genuinely poor goals |
| `min_information_gain_cells` | Minimum reduction in nearby unknown cells required for a goal to be considered useful | `2 cells` | Makes the explorer more selective about accepted exploration progress |
| `encapsulation_confirmation_cycles` | Number of consecutive enclosed-space detections required before declaring completion | `3 cycles` | Makes completion more conservative and less sensitive to transient artefacts |
| `robot_radius` | Effective robot body radius used by Nav2 costmaps for collision checking | `0.10 m` | Increases safety margin but reduces the ability to pass through tight gaps |
| `inflation_radius` | Costmap obstacle inflation distance around occupied cells | `0.35 m` | Increases path clearance, but can make narrow corridors appear less traversable |

## Design Decisions and Rationale

| Decision | Rationale |
| --- | --- |
| Use Cartographer for online SLAM instead of a prebuilt static map | The arena is unknown at mission start, so the robot must localise and build the map online. |
| Use Nav2 for path execution but not for frontier discovery | Frontier selection is a goal-generation problem, while Nav2 is better suited for planning, control, and recovery once a goal is chosen. |
| Filter frontiers with multiple quality metrics rather than choosing the nearest raw frontier | This rejects noisy, unsafe, or low-value candidates and improves real-world reliability. |
| Perform path validation before committing to a frontier | A frontier is useful only if it is reachable through the current map. |
| Use cluster-level blacklisting and failure limits | This reduces oscillation around problematic regions and avoids repeated failed goals. |
| Add an information-gain check after goal completion | Reaching a goal is not sufficient if it does not expand the explored area. |
| Add encapsulation-based completion logic | This prevents endless exploration attempts caused by tiny unreachable unknown pockets. |
| Add a post-exploration traverser with frontier reactivation | This improves residual coverage and allows the system to recover gracefully if new frontiers appear after nominal completion. |
