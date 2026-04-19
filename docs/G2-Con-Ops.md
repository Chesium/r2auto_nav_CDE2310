# Con-Ops

[Home](../README.md)

The TurtleBot3 autonomous mobile robot is intended to navigate an unknown maze, locate two target stations, and deliver three ping-pong balls to each receptacle within the 25-minute mission window.

At mission start, the operator powers on the robot and launches the required ROS 2 nodes. The mission controller enters the `INIT` state to verify that critical services and subsystems are available before enabling exploration. During `EXPLORE`, the robot performs frontier-based autonomous search while simultaneously monitoring for ArUco markers associated with Station A and Station B.

When a station is detected within operational range, the mission controller pauses exploration and transitions to docking. A visual-servo docking node uses real-time ArUco pose estimation to approach the selected station without requiring a prebuilt map for the docking manoeuvre itself. Once docking is complete, the robot hands over to a station-specific alignment subsystem.

At Station A, the alignment node uses Hough circle detection to center the launcher on the fixed receptacle before the launcher fires three balls according to the required timing sequence. At Station B, the robot first aligns to the cutout and then waits for the moving receptacle to present a valid firing window, which is detected using a rising-edge blue LED signal. Firing is triggered autonomously when the target passes the opening.

After completing delivery at one station, the robot resumes exploration to search for the remaining undelivered station. Once both stations have been served successfully, the mission controller transitions to `COMPLETE` and stops further motion. If docking or alignment fails repeatedly, the controller invokes retry logic and may skip the affected station so that the mission can continue with the remaining objectives.
