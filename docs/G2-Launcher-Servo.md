# Launcher and Servo Subsystem

[Home](../README.md)

The launcher subsystem uses a JOHO 25 kg-cm UART bus servo operated in DC motor mode and controlled by `ball_launcher_node.py` in the `g3_ball_launcher` package. Communication is performed over a USB-to-UART adapter at `115200` bps through a bundled Python SDK (`uart_sdk/`) that implements the servo packet protocol.

On startup, the node places the servo in DC mode (`MOTOR_MODE_DC = 0x00`), rotates it slowly to recover a known initial position (`CURRENT_POSITION = 4000`), and then returns to the `idle` state. When `/fire_launcher` is called, the node starts a daemon thread that issues `dc_rotate(CW, 100%)` for `2.2` seconds before calling `dc_stop()`. The launcher status progresses through `idle → firing → complete → idle`, with the `complete` state held briefly so that the mission controller can observe the transition on `/launcher_status`.

The implementation includes two layers of protection for runtime reliability. A status lock (`_lock`) prevents concurrent state corruption, while a separate UART lock (`_serial_lock`) protects access to the serial interface. A `threading.Event` is used to signal stop requests and support safe interruption of an active firing cycle.

![cad1](assets/g2-report/cad1.png)

![cad2](assets/g2-report/cad2.png)

<p align="center">Fig: Mechanical subsystem drawings</p><br>
