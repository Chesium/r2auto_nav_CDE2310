# PI Control for Docking

## What is PI Control?

PI = **Proportional-Integral** control. It's a feedback loop that adjusts the robot's speed based on:
- **P (Proportional)**: How far off target are we RIGHT NOW?
- **I (Integral)**: How much error has ACCUMULATED over time?

```
error = desired_value - measured_value
output = Kp * error + Ki * integral_of_error
```

## How It's Used for ArUco Docking

The `aruco_dock_node.py` drives the robot toward an ArUco marker using two PI loops:

### Angular Control (Turn to Face Marker)
```
bearing = arctan2(marker_x, marker_z)   # How far off-center is the marker?
angular_z = -KP_ANGULAR * bearing       # Turn toward it (P-only, no I)
```
- Positive bearing = marker is to the right → turn right (negative angular_z)
- KP_ANGULAR = 1.2

### Linear Control (Drive Forward)
```
dist_error = distance - DOCK_DIST       # How far from target stop distance?
integral += dist_error * dt             # Accumulate error over time
linear_x = KP_LINEAR * dist_error + KI_LINEAR * integral
```
- Only drives forward when bearing < 15 degrees (roughly facing marker)
- KP_LINEAR = 0.5, KI_LINEAR = 0.08

### Why Integral?

Without the I term, the robot might stop just short of the target because the P term alone produces less and less force as error decreases. The integral term "remembers" that we've been short for a while and adds a steady push.

```
Time:  1s    2s    3s    4s    5s
Error: 0.3   0.2   0.15  0.12  0.10

P only:  Slowing down... might never reach 0
P + I:   Integral builds up, pushes through to 0
```

### Integral Clamping

The integral is clamped to prevent "windup" (where it grows huge if the robot is stuck):
```python
INTEGRAL_CLAMP = 0.10
self._integral = np.clip(self._integral, -INTEGRAL_CLAMP/KI_LINEAR, INTEGRAL_CLAMP/KI_LINEAR)
```

## EMA Smoothing (Before the Controller)

Raw ArUco measurements are noisy. Before feeding values into the PI controller, they're smoothed with Exponential Moving Average:

```python
EMA_ALPHA = 0.3  # How much to trust new measurement (0=ignore, 1=trust fully)
bearing_ema = 0.3 * raw_bearing + 0.7 * previous_ema
distance_ema = 0.3 * raw_distance + 0.7 * previous_ema
```

This trades responsiveness for smoothness. Alpha=0.3 is moderately smooth.

## Done Condition

Robot stops when BOTH:
- Distance error < 4cm (`DONE_DIST_TOL = 0.04`)
- Bearing error < 3 degrees (`DONE_ANGLE_TOL = radians(3)`)

## The Station A Aligner (Different Control)

`station_a_aligner.py` uses a **simpler P-only controller** for a different task (centering a Hough-detected circle):

```
offset = circle_center_x - image_center_x    # Pixel offset
linear_vel = KP * offset                      # Only linear, no angular
```

This is pure **P-control** (no integral term) and only controls **linear** velocity (forward/backward). The mission controller handles angular alignment separately.

## Tuning Tips

| If robot... | Adjust... | How |
|-------------|-----------|-----|
| Oscillates wildly | KP too high | Decrease KP |
| Approaches too slowly | KP too low | Increase KP |
| Stops short of target | No integral or KI too low | Add/increase KI |
| Overshoots then hunts | KI too high or no clamp | Decrease KI or tighten clamp |
| Jerky movement | EMA_ALPHA too high | Decrease (more smoothing) |
| Slow to react | EMA_ALPHA too low | Increase (less smoothing) |

---

**See also:** [[g3_visual_servo]], [[ArUco Markers Explained]]
