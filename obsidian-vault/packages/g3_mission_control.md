# g3_mission_control

> **Type:** `ament_python` package
> **Purpose:** High-level mission orchestration via Finite State Machines

## Files

| File | Lines | Purpose |
|------|-------|---------|
| `mission_controller.py` | ~700 | Full competition FSM (by Daphne) |
| `simple_mission_fsm.py` | ~225 | Minimal teaching FSM |
| `mission_controller_stub.py` | ? | Placeholder/stub |
| `mission_controller_2.0.py` | ? | Older version |

## Entry Points (setup.py)

```python
'mission_controller = g3_mission_control.mission_controller:main'
'simple_mission_fsm = g3_mission_control.simple_mission_fsm:main'
```

## mission_controller.py - Deep Dive

### What It Does
Runs the entire competition mission:
1. Waits for Nav2
2. Monitors for station detections during exploration
3. Interrupts exploration to navigate to found stations
4. Aligns with receptacle using Hough circle P-controller
5. Fires 3 balls with station-specific timing
6. Moves to next station or resumes exploration

### Key Design Decisions

**Strategy: "Deliver As Found"**
- When a station is detected, immediately interrupt exploration and navigate there
- Don't wait for both stations to be found
- This minimizes total mission time

**Alignment is split across two nodes:**
- `offset_callback()` runs at camera rate (~30Hz) for smooth P-control
- `handle_alignment()` runs at 10Hz FSM tick for state management
- This separation avoids jerky movement

**Firing uses callbacks, not polling:**
- `launcher_callback("complete")` triggers delay timer
- `handle_firing()` manages the sequence
- Fire-and-forget service calls so FSM tick never blocks

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `alignment_kp` | 0.003 | P-controller gain for angular alignment |
| `max_angular_vel` | 0.5 | Maximum turn speed during alignment |
| `station_a_delay_after_ball_1` | 4.7 | Seconds to wait after ball 1 at Station A |
| `station_a_delay_after_ball_2` | 0.7 | Seconds to wait after ball 2 at Station A |
| `station_b_delay_after_ball_*` | 0.0 | No delays at Station B |

## simple_mission_fsm.py - Deep Dive

### What It Does
Demonstrates clean FSM patterns with just 4 states:
1. Wait for services to be ready
2. Enable exploration
3. When marker seen, stop exploring and dock
4. Done

### Why It Exists
- Teaching tool for clean FSM design
- Debug tool for testing exploration -> docking flow
- Simpler than full mission_controller for isolated testing

### Important Pattern: Callback Discipline
```
RULE: Callbacks ONLY write flags. They NEVER call transition_to().
```
This is the single most important design rule in the FSM. It prevents race conditions and makes the state machine deterministic - all transitions happen in one place (the 10Hz tick).

---

**See also:** [[State Machine Reference]], [[Architecture Overview]]
