# ROS1 vs ROS2: Complete Commands Reference

## Core System Commands

### Initialization and Core

| Task              | ROS1 Command        | ROS2 Command         | Notes                            |
| ----------------- | ------------------- | -------------------- | -------------------------------- |
| Start core system | `roscore`           | (automatic)          | ROS2 starts daemon automatically |
| Check core status | `roscore` running   | `ros2 daemon status` |                                  |
| Stop core         | `Ctrl+C` on roscore | `ros2 daemon stop`   |                                  |

### Workspace Management

| Task                   | ROS1 Command                 | ROS2 Command                              | Notes |
| ---------------------- | ---------------------------- | ----------------------------------------- | ----- |
| Initialize workspace   | `catkin_init_workspace`      | (automatic with colcon)                   |       |
| Build workspace        | `catkin_make`                | `colcon build`                            |       |
| Build specific package | `catkin_make --pkg PKG_NAME` | `colcon build --packages-select PKG_NAME` |       |
| Source workspace       | `source devel/setup.bash`    | `source install/setup.bash`               |       |
| Clean build            | `catkin_make clean`          | `colcon build --cmake-clean-first`        |       |

## Node and Process Management

### Node Operations

| Task               | ROS1 Command              | ROS2 Command                 | Notes |
| ------------------ | ------------------------- | ---------------------------- | ----- |
| Run a node         | `rosrun PACKAGE NODE`     | `ros2 run PACKAGE NODE`      |       |
| List running nodes | `rosnode list`            | `ros2 node list`             |       |
| Get node info      | `rosnode info /NODE_NAME` | `ros2 node info /NODE_NAME`  |       |
| Kill a node        | `rosnode kill /NODE_NAME` | `Ctrl+C` or `ros2 lifecycle` |       |

### Process Management

| Task                   | ROS1 Command                | ROS2 Command              | Notes                         |
| ---------------------- | --------------------------- | ------------------------- | ----------------------------- |
| Launch multiple nodes  | `roslaunch PKG FILE.launch` | `ros2 launch PKG FILE.py` | Major syntax change           |
| List running processes | `ps aux \| grep ros`        | `ros2 node list`          |                               |
| Monitor node status    | Custom scripts              | `ros2 component list`     | ROS2 has component management |

## Topic Operations

### Topic Information

| Task               | ROS1 Command           | ROS2 Command             | Notes                        |
| ------------------ | ---------------------- | ------------------------ | ---------------------------- |
| List all topics    | `rostopic list`        | `ros2 topic list`        |                              |
| Get topic type     | `rostopic type /TOPIC` | `ros2 topic type /TOPIC` |                              |
| Get topic info     | `rostopic info /TOPIC` | `ros2 topic info /TOPIC` | Shows publishers/subscribers |
| Monitor topic rate | `rostopic hz /TOPIC`   | `ros2 topic hz /TOPIC`   |                              |

### Topic Interaction

| Task                | ROS1 Command                    | ROS2 Command                      | Notes |
| ------------------- | ------------------------------- | --------------------------------- | ----- |
| Echo topic data     | `rostopic echo /TOPIC`          | `ros2 topic echo /TOPIC`          |       |
| Publish to topic    | `rostopic pub /TOPIC TYPE DATA` | `ros2 topic pub /TOPIC TYPE DATA` |       |
| Find topics by type | `rostopic find TYPE`            | `ros2 topic list -t \| grep TYPE` |       |

### Example Topic Commands

**ROS1:**

```bash
rostopic list
rostopic echo /virofighter/odometry
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.5, y: 0.0, z: 0.0}"
```

**ROS2:**

```bash
ros2 topic list
ros2 topic echo /virofighter/odometry
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}}"
```

## Service Operations

| Task             | ROS1 Command                    | ROS2 Command                           | Notes |
| ---------------- | ------------------------------- | -------------------------------------- | ----- |
| List services    | `rosservice list`               | `ros2 service list`                    |       |
| Get service type | `rosservice type /SERVICE`      | `ros2 service type /SERVICE`           |       |
| Call service     | `rosservice call /SERVICE ARGS` | `ros2 service call /SERVICE TYPE ARGS` |       |

### Example Service Calls

**ROS1:**

```bash
rosservice call /virofighter/get_status "{}"
```

**ROS2:**

```bash
ros2 service call /virofighter/get_status virofighter_msgs/srv/GetStatus "{}"
```

## Parameter Management

| Task            | ROS1 Command                | ROS2 Command                       | Notes                         |
| --------------- | --------------------------- | ---------------------------------- | ----------------------------- |
| List parameters | `rosparam list`             | `ros2 param list`                  |                               |
| Get parameter   | `rosparam get /PARAM`       | `ros2 param get /NODE PARAM`       | ROS2 params are node-specific |
| Set parameter   | `rosparam set /PARAM VALUE` | `ros2 param set /NODE PARAM VALUE` |                               |
| Load parameters | `rosparam load FILE.yaml`   | `ros2 param load /NODE FILE.yaml`  |                               |

### Example Parameter Operations

**ROS1:**

```bash
rosparam list
rosparam set /virofighter/max_speed 2.0
rosparam get /virofighter/max_speed
```

**ROS2:**

```bash
ros2 param list
ros2 param set /controller_node max_speed 2.0
ros2 param get /controller_node max_speed
```

## Message and Service Definitions

| Task                        | ROS1 Command       | ROS2 Command                 | Notes     |   |
| --------------------------- | ------------------ | ---------------------------- | --------- | - |
| Show message definition     | `rosmsg show TYPE` | `ros2 interface show TYPE`   |           |   |
| List all messages           | `rosmsg list`      | `ros2 interface list         | grep msg` |   |
| Find packages using message | `rosmsg packages`  | `ros2 interface package PKG` |           |   |

### Example Message Commands

**ROS1:**

```bash
rosmsg show sensor_msgs/Imu
rosmsg list | grep virofighter
```

**ROS2:**

```bash
ros2 interface show sensor_msgs/msg/Imu
ros2 interface list | grep msg
ros2 interface package virofighter_msgs
```

## Bag File Operations

| Task          | ROS1 Command           | ROS2 Command             | Notes            |
| ------------- | ---------------------- | ------------------------ | ---------------- |
| Record topics | `rosbag record TOPICS` | `ros2 bag record TOPICS` |                  |
| Play back bag | `rosbag play FILE.bag` | `ros2 bag play FILE.db3` | Different format |
| Get bag info  | `rosbag info FILE.bag` | `ros2 bag info FILE.db3` |                  |

### Example Bag Operations

**ROS1:**

```bash
rosbag record /odometry /cmd_vel
rosbag play data.bag
rosbag info data.bag
```

**ROS2:**

```bash
ros2 bag record /odometry /cmd_vel
ros2 bag play data.db3
ros2 bag info data.db3
```

## Useful Aliases for Migration

**Add to ~/.bashrc:**

```bash
# ROS1 commands (for reference)
alias r1list='rostopic list'
alias r1echo='rostopic echo'
alias r1node='rosnode list'

# ROS2 commands (primary)
alias r2list='ros2 topic list'
alias r2echo='ros2 topic echo'
alias r2node='ros2 node list'

# Build commands
alias cbuild='colcon build'
alias cbtest='colcon test'
alias cbres='colcon test-result --verbose'
```

## Quick Migration Cheat Sheet

**ROS1:**

```bash
roscore &
rosrun my_package my_node
rostopic echo /my_topic
roslaunch my_package launch_file.launch
```

**ROS2:**

```bash
# No roscore needed!
ros2 run my_package my_node
ros2 topic echo /my_topic
ros2 launch my_package launch_file.py
```

**Common Pitfalls:**

* Launch files: `.launch.py` and Python syntax
* Parameters: Node-specific, not global
* Message types: Full path required (geometry_msgs/msg/Twist)
* Build system: Use `colcon` instead of `catkin_make`
