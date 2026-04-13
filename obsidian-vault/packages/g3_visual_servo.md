# g3_visual_servo

> **Type:** `ament_python` package
> **Purpose:** ArUco marker detection, visual servoing, and docking

## Files

| File | Lines | Purpose |
|------|-------|---------|
| `aruco_dock_node.py` | ~410 | Legacy docking: detect + PI control + drive |
| `aruco_dock_pose_publisher.py` | ~293 | New: detect + publish pose for Nav2 docking |
| `dock_test.py` | ? | Manual docking test |
| `turn_calibrate.py` | ? | Turn calibration utility |
| `cmd_vel_test.py` | ? | Smoke test for velocity commands |

## Launch Files

| File | Purpose |
|------|---------|
| `aruco_dock.launch.py` | Hardware docking launch |
| `aruco_dock_hw_test.launch.py` | Hardware test with Nav2 docking server |

## Entry Points

```python
'aruco_dock = g3_visual_servo.aruco_dock_node:main'
'aruco_dock_pose_publisher = g3_visual_servo.aruco_dock_pose_publisher:main'
'dock_test = g3_visual_servo.dock_test:main'
'turn_calibrate = g3_visual_servo.turn_calibrate:main'
'cmd_vel_test = g3_visual_servo.cmd_vel_test:main'
```

---

## aruco_dock_node.py - The Legacy Docking System

### How ArUco Detection Works

1. Camera publishes images on `/usb_cam/image_raw`
2. Convert to grayscale
3. `cv2.aruco.detectMarkers()` finds marker corners
4. `cv2.solvePnP()` estimates 6DOF pose from known marker size
5. Extract bearing (arctan2) and distance (norm of tvec)

### Marker Size = 0.165m
The physical marker is 16.5cm on each side. This must be accurate or solvePnP gives wrong distance.

### Two Modes

**Passive (SCANNING):**
- Detects any marker in `{42, 67}`
- Publishes robot's map-frame pose to `/station_a_pose` or `/station_b_pose`
- Uses TF lookup (`map` -> `base_link`) to get robot position
- Does NOT move the robot

**Active (APPROACHING):**
- Triggered by `/aruco_dock/dock_to_a` or `dock_to_b` service
- Tracks only the requested marker
- PI control loop drives robot toward marker
- Stops at `DOCK_DIST = 0.30m`

### PI Control Explained

```
bearing = arctan2(marker_x_in_camera, marker_z_in_camera)
distance = norm(tvec)

angular_z = -KP_ANGULAR * bearing_ema    (turn to face marker)
linear_x  = KP_LINEAR * dist_error + KI_LINEAR * integral  (drive forward)

Only drive forward when:
  - bearing < 15 degrees (roughly facing marker)
  - distance > dock_distance (not yet close enough)
```

### EMA Smoothing
Raw solvePnP measurements are noisy. Exponential Moving Average (alpha=0.3) smooths both bearing and distance:
```
new_ema = alpha * raw_measurement + (1 - alpha) * old_ema
```

### OpenCV Compatibility
The code handles both OpenCV 4.6 (Debian) and 4.7+ APIs:
- 4.6: `aruco.Dictionary_get()`, `aruco.DetectorParameters_create()`
- 4.7+: `aruco.getPredefinedDictionary()`, `aruco.DetectorParameters()`
- Also forces arrays to be contiguous (`np.ascontiguousarray`) to avoid 4.6 segfaults

---

## aruco_dock_pose_publisher.py - The New Nav2-Compatible System

### What's Different
- **NO state machine** - purely detection -> publish
- **NO /cmd_vel** - doesn't drive the robot at all
- **NO TF lookups** - publishes in camera frame, not map frame
- Just: detect marker -> solvePnP -> publish PoseStamped on `/detected_dock_pose`

### Why This Exists
Nav2's `opennav_docking` framework (specifically `SimpleNonChargingDock` plugin) expects an external node to publish the dock's pose. The docking server handles all the approach control itself. This clean separation is better architecture.

### Key Implementation Detail: Timestamps
```python
out.header.stamp = msg.header.stamp  # Use image timestamp, NOT current time
```
The docking server has an `external_detection_timeout` - if the timestamp is stale, it considers the detection lost. Using the image's own timestamp ensures freshness checks work correctly.

### Quaternion Conversion
Since this node avoids importing `tf_transformations` (to avoid a dependency), it implements rotation-matrix-to-quaternion conversion manually using the standard trace-based algorithm.

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `image_topic` | `/camera/image_raw` | Camera feed |
| `camera_info_topic` | `/camera/camera_info` | Camera intrinsics |
| `dock_pose_topic` | `/detected_dock_pose` | Output topic |
| `target_marker_id` | 42 | Which ArUco ID to track |
| `marker_size` | 0.165 | Marker edge length (meters) |
| `dictionary` | `DICT_4X4_100` | ArUco dictionary |
| `output_frame_id` | `""` | Override frame_id (empty = use camera's) |

---

**See also:** [[ArUco Markers Explained]], [[PI Control for Docking]]
