# Frontier Exploration Explained

## The Problem

You have a robot in an unknown environment. How does it decide where to go to build a complete map?

## The Concept

An **occupancy grid** has three types of cells:
- **Free** (value 0-49): Robot can go here, we've seen it
- **Occupied** (value 50+): Wall/obstacle
- **Unknown** (value -1): Haven't looked here yet

A **frontier** is a free cell that borders at least one unknown cell. Frontiers are the "edge of knowledge" - if the robot goes there, it'll see new stuff.

```
Unknown  Unknown  Unknown
Unknown  FRONTIER  Free
Unknown  Free      Free
```

## The Algorithm (How This Project Does It)

### 1. Find All Frontier Cells
```python
for each cell in map:
    if cell is free AND any 4-neighbor is unknown:
        mark as frontier
```

### 2. Filter Low-Quality Frontiers
Not all frontiers are worth visiting. A frontier is rejected if:
- Too few free neighbors (could be a dead end)
- Too high occupied ratio nearby (too close to walls)
- Too few reachable unknown cells (won't reveal much)
- Not enough unknown span (just a tiny gap, not real unknown space)
- Not enough clearance (robot physically can't fit)

### 3. Cluster Frontiers
Adjacent frontier cells (8-connected) are grouped into clusters. Small clusters (< 10 cells) are discarded - they're probably noise.

### 4. Rank and Select
For each cluster:
1. Pick a goal cell (centroid of cluster, snapped to actual cell)
2. Ask Nav2 to compute a path (feasibility check)
3. Calculate path length

Sort by: shortest path first, then biggest cluster first.

### 5. Navigate
Send the best goal to Nav2's `NavigateToPose`. Monitor progress at 2Hz.

### 6. Handle Failures
If navigation fails:
- Blacklist the cluster for 3 minutes
- After 3 failures: permanently mark as exhausted
- Move on to next best frontier

### 7. Detect Completion
Two ways exploration ends:
1. **No frontiers found** for 5 consecutive cycles
2. **Encapsulation**: Robot's reachable space is fully enclosed by walls with only tiny unknown gaps (this prevents the robot from endlessly trying to reach unreachable areas behind walls)

## Encapsulation Detection (Smart Completion)

```
┌────────────────────┐
│ # # # # # # # # # #│
│ # . . . . . . . # #│
│ # . . R . . . . # #│  R = robot
│ # . . . . . . ? # #│  ? = tiny unknown gap
│ # # # # # # # # # #│
└────────────────────┘
```

If the free-space connected component containing the robot:
- Doesn't touch the map border
- Has very few unknown boundary cells (tiny gaps)

Then the space is "enclosed" and exploration is complete, even though some cells are still unknown.

## Visualization (RViz/Foxglove)

The explorer publishes MarkerArrays on `/exploration/frontiers`:
- **Cyan points**: Frontier cells
- **Gray spheres**: Blacklisted goals
- **Green sphere**: Current active goal

---

**See also:** [[g3g_frontier_exploration]], [[Nav2 Basics]]
