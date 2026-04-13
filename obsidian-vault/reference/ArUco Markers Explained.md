# ArUco Markers Explained

## What Are ArUco Markers?

ArUco markers are square black-and-white patterns (like simplified QR codes) that are easy for computers to detect. Each marker has a unique ID encoded in its binary pattern.

```
┌───────────────┐
│ ■ □ ■ □ ■ □ ■ │
│ □ ■ □ ■ □ ■ □ │
│ ■ □ ■ ■ □ □ ■ │  ← Binary pattern encodes ID
│ □ ■ ■ □ ■ □ □ │
│ ■ □ □ ■ □ ■ ■ │
│ □ ■ □ □ ■ ■ □ │
│ ■ □ ■ □ ■ □ ■ │
└───────────────┘
     Marker ID 42
```

## This Project Uses

| Marker ID | Dictionary | Physical Size | Purpose |
|-----------|-----------|---------------|---------|
| 42 | DICT_4X4_100 | 16.5 cm (hw) / 18 cm (sim) | Station A |
| 67 | DICT_4X4_100 | 16.5 cm | Station B |

**DICT_4X4_100** means: 4x4 internal grid, 100 possible markers in the set.

## How Detection Works

### Step 1: Find the marker in the image
```python
corners, ids, rejected = aruco.detectMarkers(gray_image, dictionary, parameters)
```
- `corners`: 4 corner points for each detected marker (pixel coordinates)
- `ids`: Which marker ID each detection is
- `rejected`: Candidate regions that didn't pass verification

### Step 2: Estimate 3D pose
```python
ok, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
```

This is the key step. Given:
- **object_points**: Known 3D positions of the 4 corners (from marker_size)
- **image_points**: Where those corners appear in the image
- **camera_matrix**: Camera's intrinsic parameters (focal length, principal point)
- **dist_coeffs**: Lens distortion coefficients

It computes:
- **rvec**: Rotation vector (how the marker is oriented)
- **tvec**: Translation vector (where the marker is in 3D space relative to camera)

```
Camera ──────tvec────────→ Marker
       [x, y, z] in meters
       
z = forward distance
x = left/right offset
y = up/down offset
```

### Step 3: Extract useful values

**Distance to marker:**
```python
distance = np.linalg.norm(tvec)  # sqrt(x² + y² + z²)
```

**Bearing (angle to marker):**
```python
bearing = arctan2(tvec[0], tvec[2])  # Angle in the horizontal plane
```

## Why Physical Marker Size Matters

`solvePnP` uses the known physical size to calculate real-world distance. If you tell it the marker is 16.5cm but it's actually 20cm, all distance measurements will be wrong.

```
Wrong marker_size = wrong distance = robot stops too far or crashes into marker
```

## Camera Calibration

The `camera_matrix` and `dist_coeffs` come from camera calibration:
- **camera_matrix** (K): 3x3 matrix with focal lengths (fx, fy) and principal point (cx, cy)
- **dist_coeffs** (D): Lens distortion parameters

Without calibration, solvePnP estimates are inaccurate. The project uses calibration files stored at:
```
/home/g3/camera_ws/src/calibration/usb_cam_calibration.yaml
```

For simulation, Gazebo provides perfect intrinsics via `/camera/camera_info`.

## OpenCV Version Gotchas

This project must work with both OpenCV 4.6 (Debian) and 4.7+:

```python
# OpenCV 4.6
dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
params = aruco.DetectorParameters_create()

# OpenCV 4.7+
dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
params = aruco.DetectorParameters()
```

Also, OpenCV 4.6 **segfaults** on non-contiguous numpy arrays, so the code uses:
```python
gray = np.ascontiguousarray(gray, dtype=np.uint8)
```

---

**See also:** [[g3_visual_servo]], [[PI Control for Docking]]
