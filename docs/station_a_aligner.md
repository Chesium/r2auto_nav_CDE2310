# g3_aligner — Station A

ROS 2 node that detects the tin receptacle circle via Hough circle detection and aligns the bot to it using Y-axis P-control. Notifies the mission controller FSM once stable alignment is achieved; firing is handled entirely by the FSM.

## Package Structure

```
g3_aligner/
├── g3_aligner/
│   ├── station_a_aligner.py   # Station A alignment node
│   └── station_b_aligner.py   # Station B alignment node
├── package.xml
└── setup.py
```

## Node: `station_a_aligner`

### Camera Orientation

The RPi camera is mounted **rotated 90° left** on the bot. As a result:

- Lateral alignment uses the **Y-axis** (`cy`) of the detected circle instead of `cx`.
- `offset_y > 0` (circle below frame centre) → bot drives **backward**
- `offset_y < 0` (circle above frame centre) → bot drives **forward**

### Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | Subscribes | Raw camera feed from RPi |
| `/receptacle/offset` | `std_msgs/Int32` | Publishes | Y-axis pixel offset from frame centre; `9999` when no circle detected |
| `/receptacle/tin_ready` | `std_msgs/Bool` | Publishes | `True` when aligned within threshold |
| `/receptacle/annotated` | `sensor_msgs/Image` | Publishes | Debug feed with alignment lines overlaid |
| `/cmd_vel` | `geometry_msgs/Twist` | Publishes | Linear X only (fwd/bwd); angular Z always `0` |

### Services Called

| Service | Type | Description |
|---|---|---|
| `/receptacle/notify_aligned` | `std_srvs/srv/Trigger` | Called once when stable alignment is reached; triggers FSM transition to `FIRE_AT_A` |

### Tuning Parameters

Defined as instance constants in `__init__`:

| Parameter | Default | Description |
|---|---|---|
| `KP_LINEAR` | `0.002` | P-gain for Y-axis linear velocity |
| `MAX_LINEAR_VEL` | `0.08` m/s | Clamp on commanded linear speed |
| `ALIGN_THRESHOLD` | `15` px | Pixel tolerance for "aligned" on Y-axis |
| `ALIGN_STABLE_FRAMES` | `5` | Consecutive aligned frames before notifying FSM |
| `CONFIRM_FRAMES` | `4` | Consecutive detection hits before circle is considered confirmed |
| `EMA_ALPHA` | `0.35` | Exponential moving average weight for `cy` smoothing |
| `DP` | `1.2` | Hough accumulator resolution ratio |
| `MIN_DIST` | `100` px | Minimum distance between detected circle centres |
| `PARAM1` | `50` | Canny high threshold for Hough |
| `PARAM2` | `25` | Accumulator threshold for circle detection |
| `MIN_R` / `MAX_R` | `55` / `75` px | Radius range, tuned for 22–38 cm distance |

## Alignment Flow

```
Camera frame received
    ↓
Hough circle detection (GaussianBlur → HoughCircles)
    ↓
Temporal filter: require CONFIRM_FRAMES consecutive hits
    ↓
EMA smoother on cy (weight = EMA_ALPHA)
    ↓
offset_y = cy_smooth - frame_centre_y
    ↓
P-control: linear_vel = clamp(KP_LINEAR * offset_y, ±MAX_LINEAR_VEL)
    publish /cmd_vel (linear.x = 0 if aligned, else linear_vel)
    ↓
If |offset_y| < ALIGN_THRESHOLD for ALIGN_STABLE_FRAMES frames
    AND service not yet called this alignment:
        call /receptacle/notify_aligned  ←── FSM transitions to FIRE_AT_A
    ↓
FSM calls /fire_launcher (handled by g3_ball_launcher)
```

If the circle is lost mid-alignment:
- `consecutive_hits` decrements each frame; `confirmed` clears at zero
- EMA state is reset — no stale position reuse
- `notified` resets on misalignment so re-alignment can re-notify

## Dependencies

- `rclpy`, `sensor_msgs`, `std_msgs`, `geometry_msgs`, `std_srvs` — ROS 2
- `cv_bridge` — ROS ↔ OpenCV image conversion
- `opencv-python` (`cv2`) — Hough circle detection
- `numpy` — image array processing

## Building

```bash
cd ~/nav_ws
colcon build --packages-select g3_aligner
source install/setup.bash
```

## Running

```bash
ros2 run g3_aligner station_a_aligner
```

## Testing

```bash
# Terminal 1 — run the node
ros2 run g3_aligner station_a_aligner

# Terminal 2 — monitor Y-axis offset
ros2 topic echo /receptacle/offset

# Terminal 3 — monitor alignment gate
ros2 topic echo /receptacle/tin_ready

# Terminal 4 — watch debug feed (requires rqt or image_view)
ros2 run image_view image_view --ros-args -r image:=/receptacle/annotated
```

Expected `/receptacle/offset` approaches `0` as the bot moves, then `/receptacle/tin_ready` publishes `True` and the FSM is notified via `/receptacle/notify_aligned`.

## Interface with Mission Controller

The FSM (`g3_mission_control`) exposes `/receptacle/notify_aligned` as a service server. On receipt:

- FSM transitions from `ALIGN_AT_A` → `FIRE_AT_A`
- FSM calls `/fire_launcher` on `g3_ball_launcher`

The aligner continues publishing `/receptacle/tin_ready` for FSM monitoring throughout.
If the notification call finds the service unavailable or returns a rejection, `notified` is reset and the call is retried on the next stable frame.
