# attach_shelf

Checkpoint 9 training package for Robot Developer Masterclass.

## Current Goal

Implement `pre_approach` learner-first:

- subscribe to `/scan`;
- extract a valid front-window distance from `sensor_msgs/msg/LaserScan`;
- publish `/cmd_vel` as `geometry_msgs/msg/Twist`;
- read `obstacle` and `degrees` parameters from launch;
- use a timer-driven state machine.

## Build

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select attach_shelf
source install/setup.bash
```

## Launch

```bash
ros2 launch attach_shelf pre_approach.launch.xml obstacle:=0.4 degrees:=-90
```

## Learner TODOs

1. Implement `get_front_distance`.
2. Implement `timer_callback` state machine.
3. Add parameter validation and safe-stop reasons.
4. Build and fix compiler errors.
5. Verify with `ros2 param get`, `/cmd_vel`, and simulated `/scan` or the checkpoint simulation.
