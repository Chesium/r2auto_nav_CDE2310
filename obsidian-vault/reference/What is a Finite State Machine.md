# What is a Finite State Machine (FSM)?

## The Core Idea

An FSM is a system that can be in exactly **one state** at any time, and transitions between states based on **events** or **conditions**.

```
[State A] --condition--> [State B] --condition--> [State C]
```

Think of it like a flowchart that the robot follows, but it can only be at one box at a time.

## Why Use FSMs in Robotics?

1. **Predictability**: You always know what the robot is doing
2. **Debuggability**: Just check the current state
3. **Safety**: Invalid transitions are impossible
4. **Modularity**: Each state is independent logic

## How It's Implemented in This Project

### The Timer-Driven Tick Pattern

```python
# Every 0.1 seconds (10Hz), run the state machine
self.create_timer(0.1, self.state_machine_tick)

def state_machine_tick(self):
    if self.state == "EXPLORE":
        self.handle_explore()
    elif self.state == "NAVIGATE":
        self.handle_navigate()
    # ... etc
```

**Why 10Hz?** Fast enough to respond quickly, slow enough to not waste CPU. The actual sensor processing (camera callbacks) runs at camera rate (~30Hz), but state decisions happen at 10Hz.

### The Callback Discipline Rule

```
CRITICAL RULE: Callbacks only WRITE data. They never change state.
```

**Why?** If a camera callback could trigger a state transition while the tick handler is also checking conditions, you get race conditions. Example:

```python
# BAD: callback changes state
def camera_callback(self, msg):
    if self.detect_marker(msg):
        self.state = "DOCK"  # RACE CONDITION!

# GOOD: callback sets flag, tick reads it
def camera_callback(self, msg):
    if self.detect_marker(msg):
        self._marker_seen = True  # Just set the flag

def tick(self):
    if self.state == "EXPLORE" and self._marker_seen:
        self.transition_to("DOCK")  # Single place for transitions
```

### Entry Guards

"On entry" actions (like starting exploration) should run **exactly once** when entering a state, not every tick:

```python
def handle_explore(self):
    if not self._explore_entered:
        self.start_exploration()     # Only runs once
        self._explore_entered = True
    
    if self._marker_seen:
        self.transition_to("DOCK")   # Runs every tick until true
```

## States in This Project

### Full Mission Controller
```
INIT → EXPLORE → NAVIGATE_TO_A → ALIGN_AT_A → FIRE_AT_A → EXPLORE → ...
```

### Simple Mission FSM
```
INIT → EXPLORE → DOCK → DONE
```

### ArUco Dock Node
```
SCANNING → APPROACHING → DONE
```

---

**See also:** [[g3_mission_control]], [[State Machine Reference]]
