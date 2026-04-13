# Improvement Ideas and Known Issues

## Known Issues

### 1. `/cmd_vel` Ownership Conflict
**Problem:** Multiple nodes publish to `/cmd_vel` simultaneously:
- `station_a_aligner` publishes linear-only commands
- `mission_controller.offset_callback()` publishes angular-only commands
- `aruco_dock_node` publishes combined commands
- Nav2 controller publishes during navigation

**Impact:** During alignment, interleaved messages mean some frames have linear but no angular, and vice versa. The "last publisher wins" on each frame.

**Fix:** Create a single velocity mux node that combines linear and angular from different sources. Or have one node aggregate both alignment axes.

### 2. Blocking `time.sleep()` in `ball_launcher_node.py`
**Problem:** The fire sequence uses `time.sleep(0.1)` on the ROS thread after setting `status = 'complete'`.

**Impact:** Minimal since it's only 100ms, but the main fire loop correctly runs in a daemon thread. The post-fire sleep on line 224 is fine.

### 3. Station B Aligner Missing
**Problem:** `station_b_aligner.py` exists but wasn't read - it likely needs different parameters since Station B has a moving receptacle.

**Todo:** Verify Station B aligner handles the moving target differently (probably needs prediction or faster loop).

### 4. `mission_controller_2.0.py` and `mission_controller_stub.py` Unused
**Problem:** These files are in the package but appear to be old versions. They add confusion.

**Fix:** Archive or remove if no longer needed.

### 5. Sim vs Hardware Camera Topic Mismatch
**Problem:** 
- Hardware camera: `/usb_cam/image_raw`
- Sim camera: `/camera/image_raw`

Nodes hardcode one or the other. `aruco_dock_node.py` uses `/usb_cam/*`, `aruco_dock_pose_publisher.py` parameterizes it.

**Fix:** Always parameterize camera topics (the new pose publisher does this correctly).

---

## Architecture Improvements

### 1. Unify Docking Approach
Currently two parallel docking systems:
- **Legacy:** `aruco_dock_node.py` (custom PI control)
- **New:** `aruco_dock_pose_publisher.py` + Nav2 docking_server

**Recommendation:** Commit fully to Nav2 docking. The pose publisher is cleaner architecture (separation of concerns). Phase out the legacy node once Nav2 docking is proven reliable.

### 2. Make `g3_receptacle_aligner` a Proper ROS2 Package
Currently standalone scripts, not a `colcon` package. No `setup.py`, no `package.xml`.

**Fix:** Create proper package structure so it can be built, launched, and parameterized properly.

### 3. Add Health Monitoring
No node monitors overall system health. If SLAM dies or Nav2 crashes, the mission controller doesn't know.

**Idea:** Add a watchdog node that monitors critical topic rates (`/map`, `/scan`, `/odom`) and triggers recovery or alerts.

### 4. Parameter Externalization
Many parameters are hardcoded constants (PID gains, Hough circle params, timing delays). 

**Fix:** Declare all tunable values as ROS parameters so they can be set from launch files or YAML configs without code changes.

---

## Performance Tuning Ideas

### Frontier Exploration
- **Increase `planning_rate_hz`** from 1.0 to 2.0 for faster replanning
- **Decrease `completion_patience_cycles`** if exploration ends too late
- **Tune `min_goal_distance`** based on room size

### ArUco Docking
- **Increase `EMA_ALPHA`** for faster response (but more noise)
- **Decrease `DOCK_DIST`** to get closer to station
- **Tune PI gains** if robot oscillates or approaches too slowly

### Ball Launcher Timing
- **Measure actual servo response time** and adjust `fire_duration`
- **Tune station_a_delays** based on actual receptacle timing

---

## Testing Checklist

- [ ] Sim: Full exploration completes without getting stuck
- [ ] Sim: Robot reaches all frontiers before declaring complete
- [ ] Sim: Docking test reaches marker within tolerance
- [ ] Sim: Nav2 docking action succeeds end-to-end
- [ ] HW: Camera publishes at expected rate
- [ ] HW: ArUco detection works at various distances
- [ ] HW: Hough circle detection rejects false positives (acrylic)
- [ ] HW: Ball launcher fires 3 balls reliably
- [ ] HW: Full mission completes (explore -> navigate -> align -> fire -> repeat)
- [ ] HW: Robot recovers from navigation failures

---

**See also:** [[Architecture Overview]], [[Package Index]]
