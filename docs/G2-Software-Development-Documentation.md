# Software Development Documentation

[Home](../README.md)

## ROS 2 Packages and Nodes

The final software stack was divided into modular ROS 2 packages so that each subsystem could be developed, tested, and integrated independently.

| Subsystem | Package(s) | Nodes | Responsibility |
| --- | --- | --- | --- |
| Mission Control | `g3_mission_control` | `mission_controller` | High-level FSM orchestration across exploration, docking, alignment, and firing |
| Visual Servo | `g3_visual_servo` | `simple_aruco_dock` | ArUco detection, station reporting, and short-range docking control |
| Exploration | `g3g_frontier_exploration` | `frontier_explorer` | Frontier selection, goal dispatch, and exploration completion logic |
| Exploration | `g3g_frontier_exploration` | `post_exploration_traverser` | Residual free-space traversal after frontier exhaustion |
| Receptacle Alignment | `g3_aligner` | `station_a_aligner` | Hough-based alignment for the static receptacle at Station A |
| Receptacle Alignment | `g3_aligner` | `station_b_aligner` | Hough alignment plus blue-LED-triggered firing for Station B |
| Ball Launcher | `g3_ball_launcher` | `ball_launcher_node` | UART servo control for timed firing and stop/status reporting |
| Navigation | `g3nav2` | Cartographer, RViz, Nav2 lifecycle nodes | Real-hardware launch files, navigation configuration, and SLAM integration |
| Simulation | `g3gzsim` | Gazebo and ROS-GZ support nodes | Simulation worlds, robot models, and sim-specific launch/configuration files |

## Development Environment, Networking Strategy, and Debugging Workflow

This section summarises the main tools, workflows, and engineering conventions adopted during development. These choices were driven not only by convenience, but also by the realities of building a distributed ROS 2 system across a TurtleBot Raspberry Pi and an operator laptop.

### Networking and System Configuration

Early in the project, mobile-phone hotspots were suggested as the default networking approach. In practice, this proved unsuitable for sustained development and debugging. The arrangement was fragile, required the phone to remain physically present and powered, and introduced extra latency and bandwidth limitations. It also made network reconfiguration cumbersome because the Raspberry Pi often had to be reconnected to a monitor and keyboard for manual changes.

To improve this setup, several changes were made. First, Ubuntu Desktop was installed on the Raspberry Pi instead of Ubuntu Server. This simplified network configuration, especially when connecting to the campus `NUS_STU` Wi-Fi network with WPA2/PEAP authentication. Second, the Raspberry Pi was linked to a dedicated Tailscale account so that the robot remained remotely reachable even when it was not on the same local network as the operator laptop.

Because ROS 2 communication over Tailscale alone was not fully satisfactory, Husarnet combined with `ROS_DISCOVERY_SERVER` was also evaluated as a fallback remote-debugging option. Although this worked for lightweight ROS traffic, video latency and bandwidth limitations made it unsuitable as the primary workflow.

The final preferred setup was to connect the Raspberry Pi directly to the operator laptop's Wi-Fi hotspot while providing the laptop with separate Internet access through Ethernet when needed. In this configuration, the Raspberry Pi could still reach Tailscale through `NUS_STU` at boot, after which `nmcli` was used to switch it onto the laptop hotspot for lower-latency ROS communication. This became the final operational network topology used during deployment.

### ROS 2 Version and Development Platform

During experimentation with remote ROS workflows, references suggested that the `ROS_DISCOVERY_SERVER` approach was not ideal for ROS 2 Humble in this use case. Subsequent testing also showed that ROS 2 Jazzy behaved more reliably for Gazebo installation and simulation. For these reasons, the software development environment was standardised on ROS 2 Jazzy.

To support development across Ubuntu 22.04 and Windows WSL hosts, the team adopted a Docker-based workflow for development and simulation. This reduced onboarding friction and improved reproducibility by avoiding manual recreation of dependencies on each host system. For deployment on the robot, however, the software stack was run natively on Ubuntu 24.04 for better compatibility and operational simplicity.

### Integration and Debugging Workflow

As the project matured, the number of interacting subsystems increased significantly. Running the entire stack through a single long launch command became increasingly inconvenient because it made it difficult to isolate logs, restart individual components, or inspect failures in real time.

To address this, the team created a set of `tmux` scripts that automatically opened a 2x2 panel layout and prefilled each pane with the appropriate startup commands for selected subsystems. Grouping panes by subsystem rather than by machine or launch stage made the setup easier to understand during integration and allowed multiple team members to inspect different parts of the system simultaneously.

![tmux](assets/g2-report/tmux.png)

<p align="center">Fig: One of the `tmux` launch layouts used during integration and debugging</p><br>

### Visualisation and Runtime Monitoring

In addition to RViz, Foxglove was adopted as a dashboard for monitoring camera streams and other ROS 2 topics. Its tab-based layout made it straightforward to maintain separate workspaces for navigation, perception, and system-level monitoring. Compared with RViz, Foxglove was especially effective for building multi-panel views quickly and for inspecting fields within structured ROS messages during subsystem debugging.

![foxglove1](assets/g2-report/foxglove1.png)

<p align="center">Fig: Foxglove dashboard used during runtime monitoring</p><br>
