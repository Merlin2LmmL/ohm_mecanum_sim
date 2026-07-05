# ohm_mecanum_sim

A pygame-based 2D robot simulator for mecanum-driven kinematic concepts, built for ROS 2.

![Screenshot of Robot Simulator](/images/screenshot.png)

## Setup

```bash
# 1. Clone into your workspace
git clone https://github.com/Merlin2LmmL/ohm_mecanum_sim.git ~/ros2_ws/src/ohm_mecanum_sim

# 2. Install dependencies (ROS + system packages, including pygame, via rosdep)
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 3. Build
colcon build --symlink-install

# 4. Source the workspace
source install/setup.bash
```

`rosdep` pulls in pygame and every other system dependency automatically, so there's no separate `pip3 install pygame` step.

## Running the simulator

```bash
ros2 run ohm_mecanum_sim ohm_mecanum_sim_node
```

## Configuring robots and the arena

Robot count, spawn poses, screen size, and arena wall thickness are parameters on the `ohm_mecanum_sim` node.

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `robot_names` | string array | `["robot1"]` | Namespace for each robot |
| `robot_x` | double array | `[2.0]` | Spawn x, meters |
| `robot_y` | double array | `[2.0]` | Spawn y, meters |
| `robot_theta` | double array | `[0.0]` | Spawn heading, radians |
| `screen_width` | int | `1600` | Window width, pixels |
| `screen_height` | int | `900` | Window height, pixels |
| `outer_border` | int | `5` | Outer arena wall offset, pixels |
| `inner_border` | int | `300` | Inner arena wall offset, pixels |

`robot_names`, `robot_x`, `robot_y`, and `robot_theta` are parallel arrays, one entry per robot. They must all be the same length, or the node raises an error at startup.

Each spawned robot is itself a node, named after its entry in `robot_names`, and takes its own parameters for sensing and kinematics:

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `laserbeams` | int | `36` | Number of simulated laser beams, spread evenly over 360° |
| `lasernoise` | double | `0.02` | Gaussian noise stddev on laser range readings, meters |
| `laser_range` | double | `8.0` | Maximum laser range, meters |
| `wheel_radius` | double | `0.05` | Wheel radius, meters |
| `wheel_omega_max` | double | `10.0` | Maximum wheel angular rate, rad/s |
| `wheel_base` | double | `0.3` | Front-to-back wheel distance, meters |
| `track` | double | `0.2` | Left-to-right wheel distance, meters |
| `zoomfactor` | double | `1.0` | Scale factor for the robot's on-screen image |
| `image_normal` | string | `mecanum_ohm_1.png` | Filename of the primary driving-animation frame |
| `image_alt` | string | `mecanum_ohm_2.png` | Filename of the alternate driving-animation frame |
| `image_crash` | string | `mecanum_crash_2.png` | Filename shown after a collision reset |

Image filenames are resolved against the package's installed `images` share directory, so a custom image has to live there (or you point `image_normal`/`image_alt`/`image_crash` at whatever filename you've added to that directory) rather than an arbitrary path on disk.

Put node-level and per-robot parameters in one file. `example_params.yaml`, included in the repo:

```yaml
ohm_mecanum_sim:
  ros__parameters:
    screen_width: 1600
    screen_height: 900
    outer_border: 5
    inner_border: 300
    robot_names: ["robot1", "robot2"]
    robot_x: [2.0, 5.0]
    robot_y: [2.0, 2.0]
    robot_theta: [0.0, 3.14]

robot1:
  ros__parameters:
    laserbeams: 36
    lasernoise: 0.02
    laser_range: 8.0
    wheel_radius: 0.05
    wheel_omega_max: 10.0
    wheel_base: 0.3
    track: 0.2
    zoomfactor: 1.0

robot2:
  ros__parameters:
    laserbeams: 16
    lasernoise: 0.05
    laser_range: 5.0
    wheel_radius: 0.05
    wheel_omega_max: 6.0
    wheel_base: 0.3
    track: 0.2
    zoomfactor: 1.2
```

```bash
ros2 run ohm_mecanum_sim ohm_mecanum_sim_node --ros-args --params-file src/ohm_mecanum_sim/example_params.yaml
```

> `--params-file` is resolved against the current working directory you launch from, not the package's install or source directory. The path above assumes you're running from `~/ros2_ws`. A missing file surfaces as `Error opening YAML file` underneath a `Couldn't parse params file` line, that second line is just `rcl` echoing the flag and value back for context, not a sign the argument itself was malformed.

> Keep numeric arrays as floats with a decimal point (`2.0`, not `2`). ROS 2 infers array type from the YAML values, and an all-integer list gets typed as an integer array, which then fails to match the double array the node expects.

For a one-off override without a file:

```bash
ros2 run ohm_mecanum_sim ohm_mecanum_sim_node --ros-args -p robot_names:="['robot1', 'robot2']" -p robot_x:="[2.0, 5.0]" -p robot_y:="[2.0, 2.0]" -p robot_theta:="[0.0, 0.0]"
```

## Controlling the robots

Each robot listens on its own namespaced topics, so you can drive `robot1`, `robot2`, etc. independently.

**Joystick**, via the `joy` package:
```bash
ros2 run joy joy_node --ros-args --remap joy:=/robot1/joy
```

**Manual twist commands**, e.g. a straight forward move:
```bash
ros2 topic pub -r 10 /robot1/cmd_vel geometry_msgs/Twist "{linear: {x: 1.0}}"
```

**Keyboard**, via `teleop_twist_keyboard`:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/robot1/cmd_vel
```

Or publish to `cmd_vel`/`joy` from your own node, that's the whole interface.

## Tested platforms

Primarily tested with ROS 2 Jazzy. ROS 2 Humble also works.
