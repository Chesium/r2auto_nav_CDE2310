# Testing Documentation

[Home](../README.md)

This section summarises the main tests carried out while preparing for the graded run. The tests focused on launcher reliability, receptacle alignment, and early validation of the navigation and exploration stack.

## Launcher and Mechanical

The launcher subsystem required extensive empirical testing because the physical behaviour of the servo and cam mechanism did not always match the expected motion from the documentation. In particular, while the hardware nominally supported counterclockwise motion in servo mode, the final design required continuous DC-mode rotation, which introduced practical constraints and forced a redesign of the cam geometry.

### 1. Proof-of-Concept Test

**Aim:** Verify that the cam-based mechanism could launch ping-pong balls over a distance of approximately 20-30 cm.

**Outcome:** The mechanism successfully launched the balls, confirming that the concept was viable.

![launch15](assets/g2-report/launcher_proofofconcept.png)

### 2. Isolated Test with Tin

**Aim:** Evaluate whether the launcher could consistently score into a stationary tin receptacle.

**Outcome:** The system was able to score, but the firing result was inconsistent.

**Action:** A follow-up diagnostic test was conducted to identify the sources of inconsistency before repeating the isolated firing test.

![launch10](assets/g2-report/launcher_isolatedtestwithtin.png)

### 3. Inconsistency Check Test

**Aim:** Identify the causes of inconsistent launcher behaviour.

**Outcome:** Three primary issues were identified:

- Ball position within the launcher was not repeatable.
- The two cams had uneven edges, causing one side to release before the other and driving the striker at an angle.
- Tape inside the launcher interfered with striker motion and introduced additional variation in shot direction.

**Action:** The team added tape to stabilise ball positioning, filed one cam edge to better match the other, and repositioned the internal tape to keep it outside the striker path.

![launch7](assets/g2-report/launcher_inconsistencycheck.png)

### 4. Station A Test

**Aim:** Evaluate launcher performance using the actual maze tin geometry for Station A.

**Outcome:** The receptacle height in the real setup was higher than assumed during the isolated tin test, so the ball repeatedly struck the lower rim.

**Action:** The launcher angle was adjusted and the earlier firing tests were repeated with the revised geometry.

![dock1](assets/g2-report/launcher_stationatest.png)

<p align="center">Fig: Station A launcher test before and after angle adjustment</p><br>

### 5. Alignment with RPi Camera Integration Test

**Aim:** Verify that the launcher remained accurate when alignment was driven by the RPi camera and Hough-based receptacle detection.

**Outcome:** The test was successful. The cam position was adjusted until the launcher remained accurate under vision-guided alignment.

![launch13](assets/g2-report/launcher_alignmenttest.png)

## Receptacle Alignment

### Station A

**Test A-1: Hough Circle Parameter Calibration**

The objective was to identify suitable values for `MIN_R`, `MAX_R`, and `PARAM2` over the required stand-off range of 22-38 cm. The robot was positioned at measured distances within this range while the aligner node was executed. Circle radii and detection quality were monitored through the annotated image feed, and the parameters were tuned iteratively until stable detection was achieved. The final settings, `MIN_R = 80`, `MAX_R = 150`, and `PARAM2 = 45`, provided reliable receptacle detection while suppressing false circles from the acrylic maze walls.

**Test A-2: P-Controller Convergence**

The robot was placed approximately 10 cm out of alignment before the Station A aligner was activated. The time required to reach the `ALIGNED` condition and satisfy the stable-frame threshold was recorded. The controller converged in approximately 2 seconds at a stand-off distance of 30 cm with no visible oscillation.

### Station B

**Test B-1: HSV Threshold Calibration**

The objective was to calibrate HSV thresholds for blue LED detection. The LED target was powered while the side camera observed the moving receptacle. Hue, saturation, and value limits were adjusted until the detected LED area exceeded `200 px^2` when visible and remained below `20 px^2` when obscured. The final thresholds, `H[100-130]`, `S[180-255]`, and `V[200-255]`, produced clean segmentation. Visible LED area typically ranged from `400-800 px^2`, while obscured states remained below `10 px^2`.

**Test B-2: Cutout Alignment Validation**

The objective was to confirm that Phase 1 alignment reliably targeted the circular cutout rather than the moving receptacle itself. With the receptacle placed at multiple positions, the aligner was run repeatedly and consistently locked onto the cutout centroid. Hough detection remained reliable across the 22-38 cm operating range, and alignment was typically achieved within five stable frames.

**Test B-3: Rising-Edge Trigger Validation**

The receptacle was moved manually past the cutout while launch events were monitored. With `LED_CONFIRM_FRAMES = 30`, the debounce logic prevented double-triggering, and each pass of the receptacle generated exactly one fire event.

## Navigation and Exploration Test in Simulation

Before committing fully to hardware tests, the team performed an early logic validation of the frontier exploration node in Gazebo using the `simple_colored_warehouse` environment. This helped identify several core issues, including frontier detection outside wall boundaries and an infinite-loop behaviour that was later mitigated by the addition of the information-gain check.

Although the simulator was useful for early debugging, the team transitioned quickly to real-hardware testing because of the large gap between the simulated and physical environments. During these tests, it was also observed that node processes launched by Cartographer and Nav2 could remain alive after `Ctrl-C`, which sometimes interfered with subsequent runs. To streamline later tests, the team created one-line cleanup commands and shell aliases to terminate residual processes reliably.

![sim2](assets/g2-report/sim2.png)

<p align="center">Fig: Gazebo environment used for early exploration testing</p><br>

![sim1](assets/g2-report/sim1.png)

<p align="center">Fig: RViz view showing frontiers detected during the initial simulation tests</p><br>

## Camera and Servo Integration Issues

Integration testing also highlighted several practical issues around power and configuration. The launcher was tested with a bench power supply to estimate current requirements and validate the electrical budget. This confirmed that the Raspberry Pi power pins could support the servo current draw of approximately `300 mA`, removing the need for a separate `5V` DC-DC converter in the final configuration.

These tests also reinforced the importance of checking parameter flags carefully before each run, as incorrect launch or runtime configuration could mask otherwise functional subsystem behaviour.
