# g3_ball_launcher

ROS 2 driver node that receives fire commands from the mission controller and drives a UART bus servo in DC motor mode to launch balls.

## Package Structure

```
g3_ball_launcher/
├── g3_ball_launcher/
│   ├── ball_launcher_node.py   # Main ROS 2 node
│   └── uart_sdk/               # Bundled UART servo SDK (JOHO)
│       ├── uart_servo.py       # UartServoManager class
│       ├── packet.py           # Packet framing/parsing
│       ├── packet_buffer.py    # Serial receive buffer
│       └── data_table.py       # Register addresses and constants
├── package.xml
└── setup.py
```

## Node: `ball_launcher_node`

### Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/fire_launcher` | `std_msgs/Bool` | Subscribes | Receive `True` from mission controller to trigger one shot |
| `/launcher_status` | `std_msgs/String` | Publishes | Reports current launcher state at 10 Hz |

### Launcher Status Values

| Value | Meaning |
|---|---|
| `idle` | Ready to accept a fire command |
| `firing` | Motor spinning, ball being launched |
| `complete` | Shot done, notifying mission controller |

### Hardware Config

Defined as constants at the top of `ball_launcher_node.py`:

| Constant | Default | Description |
|---|---|---|
| `SERVO_PORT` | `/dev/ttyUSB0` | USB serial port the servo is connected to |
| `SERVO_BAUD` | `115200` | Baud rate (must match servo configuration) |
| `SERVO_ID` | `1` | Servo ID on the UART bus |
| `FIRE_DURATION` | `2.2` | Seconds to spin motor per ball |

## Fire Sequence

```
Mission controller publishes True → /fire_launcher
    ↓
fire_cb() triggered
    ↓
status = 'firing'  (blocks further fire commands)
    ↓
Background thread starts:
    dc_rotate(CW, PWM=100)
    sleep(FIRE_DURATION)
    dc_stop()
    ↓
status = 'complete'  (mission controller increments ball count)
    ↓
sleep(0.1s)
    ↓
status = 'idle'  (ready for next ball)
```

## Dependencies

- `rclpy`, `std_msgs` — ROS 2
- `python3-serial` (`pyserial`) — UART serial communication

### Installing pyserial

```bash
# via apt (preferred)
sudo apt install python3-serial

# or via pip inside dev container
pip install pyserial --break-system-packages
```

## Building

```bash
cd ~/nav_ws
colcon build --packages-select g3_ball_launcher
source install/setup.bash
```

## Running

```bash
ros2 run g3_ball_launcher ball_launcher_node
```

## Testing

In separate terminals:

```bash
# Terminal 1 — run the node
ros2 run g3_ball_launcher ball_launcher_node

# Terminal 2 — monitor status
ros2 topic echo /launcher_status

# Terminal 3 — trigger a shot
ros2 topic pub --once /fire_launcher std_msgs/msg/Bool "{data: true}"
```

Expected output on `/launcher_status`: `idle` → `firing` → `complete` → `idle`

## Interface with Mission Controller

The mission controller (`g3_mission_control`) checks both conditions before firing:
```python
launcher_ready == True AND launcher_status == 'idle'
```

- `launcher_ready` is set `False` when a fire command is sent, and `True` when `complete` is received
- The node **must** return to `idle` after `complete` or the mission controller will not fire the next ball

Inter-ball delays (e.g. 7s after ball 1 at Station A) are handled entirely by the mission controller — this node does not need to implement them.
