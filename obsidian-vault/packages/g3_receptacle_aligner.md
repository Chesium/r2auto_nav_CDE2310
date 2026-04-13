# g3_receptacle_aligner

> **Type:** Standalone Python scripts (not a proper ROS2 package)
> **Purpose:** Hough circle detection to align robot with tin receptacles at stations

## Files

| File | Purpose |
|------|---------|
| `station_a_aligner.py` | Linear P-control alignment for Station A (static tin) |
| `station_b_aligner.py` | Alignment for Station B (moving receptacle) |

## station_a_aligner.py - Deep Dive

### What It Does
Detects the circular tin opening using Hough circle detection, then drives the robot forward/backward (linear only, no turning) to center the circle in the camera frame.

### Detection Pipeline

```
Compressed Image (/camera/image_raw/compressed)
    │
    ▼
JPEG decode → BGR frame
    │
    ▼
Grayscale → GaussianBlur (11x11, sigma=2)
    │
    ▼
HoughCircles (dp=1.2, minDist=100, param1=50, param2=25, r=41-61)
    │
    ▼
Pick largest circle → Darkness check (reject bright acrylic)
    │
    ▼
Temporal filter (4 consecutive hits to confirm)
    │
    ▼
EMA smoothing (alpha=0.35) on center x-coordinate
    │
    ▼
P-control: linear_vel = KP * (cx - center_x)
```

### Darkness Check
**Problem:** Hough circles also detect reflections on acrylic walls.
**Solution:** Sample the interior of detected circle. Real tin is dark (mean gray < 100). Acrylic is bright. Reject circles with bright interiors.

```python
# Sample inner 60% of radius
sr = max(3, int(r * 0.6))
mask = np.zeros(gray.shape, dtype=np.uint8)
cv2.circle(mask, (cx, cy), sr, 255, -1)
mean_inside = cv2.mean(gray, mask=mask)[0]
if mean_inside > 100:  # too bright = acrylic, not tin
    reject
```

### P-Control

Only **linear** (forward/backward) control. No rotation.
- `offset > 0` (tin is right of center) -> drive forward
- `offset < 0` (tin is left of center) -> drive backward
- `|offset| < 15px` -> aligned, stop

```python
KP_LINEAR = 0.002
MAX_LINEAR_VEL = 0.08
linear_vel = clamp(KP * offset, -0.08, 0.08)
```

### Published Topics

| Topic | Type | Rate | Purpose |
|-------|------|------|---------|
| `/receptacle/offset` | Int32 | ~30Hz | Pixel offset (9999 = not detected) |
| `/receptacle/aligned` | Bool | ~30Hz | True when centered |
| `/receptacle/tin_ready` | Bool | ~30Hz | Same as aligned for Station A |
| `/receptacle/annotated` | Image | ~30Hz | Debug overlay |
| `/cmd_vel` | Twist | ~30Hz | Linear-only velocity |

### Parameters (Hardcoded)

| Param | Value | Purpose |
|-------|-------|---------|
| `KP_LINEAR` | 0.002 | Pixels to m/s gain |
| `MAX_LINEAR_VEL` | 0.08 m/s | Speed cap |
| `ALIGN_THRESHOLD` | 15 px | "Aligned" if offset within this |
| `MIN_R` / `MAX_R` | 41 / 61 px | Hough circle radius range |
| `CONFIRM_FRAMES` | 4 | Consecutive detections required |
| `EMA_ALPHA` | 0.35 | Smoothing factor |
| `DARKNESS_THRESHOLD` | 100 | Max mean gray for valid circle |

### Integration with Mission Controller
The mission controller's `offset_callback()` ALSO does P-control (angular, not linear) based on `/receptacle/offset`. There's a potential conflict:
- `station_a_aligner` publishes linear commands on `/cmd_vel`
- `mission_controller` publishes angular commands on `/cmd_vel`
- Both run simultaneously during ALIGN_AT_A state

This works because the aligner only sets `linear.x` and the mission controller only sets `angular.z` -- but they publish separate Twist messages, so **the last one wins**. In practice, the aligner publishes at camera rate (~30Hz) and the mission controller at 10Hz + camera rate, so commands interleave.

> **Potential Issue:** This interleaving means some `/cmd_vel` messages have linear but no angular, and vice versa. A proper design would have a single node combining both.

---

**See also:** [[g3_mission_control]], [[State Machine Reference]]
