# Con-Ops

[Home](../README.md)

The TurtleBot3 Autonomous Mobile Robot is designed to autonomously navigate an unknown maze environment and deliver three ping-pong balls into each of two target receptacle stations within a 25-minute mission window.

At mission start, the operator powers on the robot and launches all required ROS 2 nodes. The mission controller enters the INIT state to verify service readiness before enabling exploration mode. During exploration, the robot performs frontier-based autonomous search while simultaneously detecting ArUco markers corresponding to Station A and Station B.

When a station is detected within operational range, exploration is paused and the robot transitions into docking mode. A visual-servo docking controller uses real-time ArUco pose estimation to approach the target station accurately without requiring Nav2 localisation or a prebuilt map.

After docking, the robot performs final alignment using a station-specific perception node. For Station A, Hough Circle detection aligns the launcher with the fixed receptacle before sequentially firing three balls with programmed delays. For Station B, the robot aligns to the cutout and autonomously fires based on rising-edge blue LED detection as the moving receptacle passes the firing window.

Once delivery at one station is completed, the robot resumes exploration to locate the remaining undelivered station. After successful delivery to both stations, the mission controller transitions to COMPLETE state and stops all motion.

If alignment or docking fails, retry logic is invoked up to a predefined maximum before skipping to the next available station.
