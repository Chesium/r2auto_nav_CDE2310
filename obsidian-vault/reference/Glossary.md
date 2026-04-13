# Glossary

| Term | Meaning |
|------|---------|
| **ArUco** | A type of fiducial marker (square pattern) easily detected by OpenCV |
| **BT (Behavior Tree)** | Nav2's task orchestration system (like a fancy FSM) |
| **Cartographer** | Google's SLAM algorithm used for 2D mapping |
| **cmd_vel** | ROS topic for velocity commands (linear + angular) |
| **colcon** | ROS 2 build tool (like make/cmake wrapper) |
| **Costmap** | Grid showing navigation cost (free=low, obstacle=high, inflated=medium) |
| **cv_bridge** | Converts between ROS Image messages and OpenCV numpy arrays |
| **Docker** | Container tool - this project runs in Docker on Ubuntu/WSL2 |
| **EMA** | Exponential Moving Average - smoothing filter for noisy data |
| **FastDDS** | The default DDS (data distribution) middleware for ROS 2 |
| **Foxglove** | Web-based visualization tool (alternative to RViz) |
| **Frontier** | Boundary between known free space and unknown space |
| **FSM** | Finite State Machine - system in exactly one state at a time |
| **Gazebo Harmonic** | Physics simulator for testing robots virtually |
| **Hough Circle** | OpenCV algorithm to detect circles in images |
| **Jazzy** | ROS 2 distribution (like Ubuntu version for ROS) |
| **Nav2** | ROS 2 navigation framework (path planning, obstacle avoidance) |
| **OccupancyGrid** | 2D grid map where each cell = free/occupied/unknown |
| **PI Control** | Proportional-Integral controller (feedback control loop) |
| **PoseStamped** | ROS message: position (x,y,z) + orientation (quaternion) + timestamp |
| **QoS** | Quality of Service - ROS 2 message delivery guarantees |
| **Quaternion** | 4-number rotation representation (x,y,z,w) - avoids gimbal lock |
| **RViz** | ROS visualization tool for 3D data |
| **SDF** | Simulation Description Format - Gazebo world/model format |
| **SLAM** | Simultaneous Localization And Mapping |
| **solvePnP** | OpenCV function: known 3D points + 2D image points → 3D pose |
| **TF** | ROS Transform system - tracks coordinate frame relationships |
| **TRANSIENT_LOCAL** | QoS durability that "latches" - new subscribers get last message |
| **TurtleBot3** | Small educational robot platform (we use the Burger variant) |
| **Twist** | ROS message for velocity: linear (x,y,z) + angular (x,y,z) |
| **TwistStamped** | Twist + header (timestamp + frame_id) |
| **UART** | Serial communication protocol (used for servo motor) |
| **URDF/Xacro** | Robot description formats (joints, links, sensors) |
| **XACRO** | XML macro language that generates URDF/SDF |
