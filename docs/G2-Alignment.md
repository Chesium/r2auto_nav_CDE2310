# Receptacle Alignment Subsystem

[Home](../README.md)

## Overview

The receptacle alignment subsystem uses a side-mounted Raspberry Pi Camera V2, rotated 90° counter-clockwise, to detect the circular receptacle opening and align the launcher before firing. Two nodes are used: one for Station A and one for Station B. Both rely on Hough-based geometric alignment, but they differ in how the final firing event is triggered.

Because the camera is physically rotated relative to the robot drive axis, lateral launcher alignment maps to the image `Y` axis rather than the image `X` axis. As a result, all alignment offsets and proportional-control outputs are computed using `cy` rather than `cx`.

![align7](assets/g2-report/align7.png)

<p align="center">Fig: Hough circle detection, Y-axis P-control, and launcher monitoring in Foxglove</p><br>

## Station A: Static Receptacle Alignment

For Station A, the alignment process combines Hough circle detection with a proportional controller on the image `Y` axis.

### Processing Pipeline

For each incoming frame, the node performs the following sequence:

1. Decode the compressed image from `/camera/image_raw/compressed`.
2. Convert the frame to grayscale and apply Gaussian blur.
3. Run `HoughCircles(HOUGH_GRADIENT)` to detect candidate circular openings.
4. Select the largest detected circle as the most likely receptacle.
5. Compute the vertical offset, `offset_y = cy_detected - frame_height / 2`.
6. Smooth the detected center using an exponential moving average.
7. Apply proportional control to generate `linear.x`, clamped to the configured velocity limit.
8. Publish `/cmd_vel` with linear motion only.
9. Count consecutive aligned frames and call `/receptacle/notify_aligned` once the stability threshold is met.

### Tuned Parameters

The following parameters were tuned using physical tests at stand-off distances of approximately 22-38 cm:

| Parameter | Value | Effect |
| --- | --- | --- |
| `Kp` (linear) | `0.0015` | Lower values produce smoother but slower convergence |
| `MAX_LINEAR_VEL` | `0.08 m/s` | Prevents overshoot at short range |
| `ALIGN_THRESHOLD` | `15 px` | Offset band treated as aligned |
| `ALIGN_STABLE_FRAMES` | `5` | Number of consecutive aligned frames required before notifying the FSM |
| `CONFIRM_FRAMES` | `5` | Number of consecutive detections required before a circle is accepted |
| `EMA_ALPHA` | `0.20` | Heavier smoothing reduces jitter |
| `MIN_R / MAX_R` | `80 / 150 px` | Radius window tuned to the operating distance |
| `PARAM2` | `45` | Hough accumulator threshold balancing sensitivity and false positives |

![align6](assets/g2-report/align6.png)

<p align="center">Fig: Station A alignment commanding backward motion at approximately 35 cm</p><br>

![align4](assets/g2-report/align4.png)

<p align="center">Fig: Station A alignment commanding forward motion at approximately 35 cm</p><br>

![align5](assets/g2-report/align5.png)

<p align="center">Fig: Station A alignment at close range after convergence</p><br>

## Station B: Moving Receptacle Alignment

Station B presents a moving receptacle on a motorised rail, so the subsystem operates in two phases.

### Phase 1: Geometric Alignment

The robot first aligns to the wooden cutout using the same Hough-based `Y`-axis alignment strategy as Station A. Once the alignment condition is satisfied for the required number of consecutive frames, the robot is considered locked and translational motion is set to zero.

### Phase 2: Blue LED Trigger Detection

After geometric alignment, the node waits for the blue LED inside the moving receptacle to become visible through the cutout. LED detection is performed within the Hough-detected circular region of interest using HSV thresholding:

- `H = 100-130`
- `S = 180-255`
- `V = 200-255`

The detection ratio is defined as the number of blue pixels divided by the circle area. A firing event is confirmed only after 30 consecutive positive frames, which suppresses false triggers caused by rail vibration or transient reflections. Each rising edge in LED visibility triggers a `/fire_launcher` call, with `BALLS_TO_FIRE = 1` per event.

### Station B Parameter Adjustments

Most alignment parameters are shared with Station A. The following values were adjusted specifically for Station B:

| Parameter | Value | Remarks |
| --- | --- | --- |
| `EMA_ALPHA` | `0.15` | Stronger smoothing for a noisier visual condition |
| `CONFIRM_FRAMES` | `8` | More frames required before locking onto a valid detection |

### Rationale for HSV Detection

HSV-based detection was selected because brightness-only methods proved unreliable during testing.

1. Reflections inside the receptacle produced brightness values similar to the aligned state even when the receptacle was not properly positioned.
2. Dark objects behind the cutout produced false positives when thresholding was based only on intensity.

By isolating a specific hue range, HSV thresholding was substantially more robust to ambient illumination changes and background variation.

![align1](assets/g2-report/align1.png)

<p align="center">Fig: Brightness-threshold failure case for Station B</p><br>

![align3](assets/g2-report/align3.png)

<p align="center">Fig: Dark-background false positive case for Station B</p><br>
