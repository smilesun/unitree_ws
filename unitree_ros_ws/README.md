# Unitree ROS Workspace

This workspace includes a Z1 SDK ROS wrapper and custom messages for low-level torque control.

## Packages Added
- `unitree_z1_msgs`: ROS message definitions for Z1 low-level commands.
- `z1_sdk_ros`: ROS node that wraps the Unitree Z1 SDK and sends low-level commands to real hardware.

## Message: `unitree_z1_msgs/LowCmd`
Fixed-length arrays of 7 (6 joints + gripper):
- `q`   joint position command (rad), gripper in index 6
- `qd`  joint velocity command (rad/s)
- `tau` joint torque feedforward (Nm)
- `kp`  joint position gains
- `kd`  joint velocity gains

Where it is used:
- `unitree_ws/unitree_ros_ws/src/z1_sdk_ros/src/z1_lowcmd_node.cpp` includes and subscribes to `unitree_z1_msgs/LowCmd`.
- `unitree_ws/unitree_ros_ws/src/z1_sdk_ros/CMakeLists.txt` and `unitree_ws/unitree_ros_ws/src/z1_sdk_ros/package.xml` declare the dependency.

Important: `unitree_z1_msgs/LowCmd` is a **message type**, not a topic.  
The **topic name** is `/z1/lowcmd`. Any node can subscribe to `/z1/lowcmd` as long as it uses the same message type.

## Build
```bash
cd /home/sunxd/robustCapture/unitree_ws/unitree_ros_ws
catkin_make
```

## Run Z1 Low-Level Command Node
```bash
source /home/sunxd/robustCapture/unitree_ws/unitree_ros_ws/devel/setup.bash
roslaunch z1_sdk_ros z1_lowcmd.launch
```

State output:
- Publishes `sensor_msgs/JointState` on `/z1/joint_states` (configurable via `~state_topic`).
- Joint names can be overridden with `~joint_names` (size 7).

## Minimal Publisher (Test)
This publishes a constant `unitree_z1_msgs/LowCmd` message to `/z1/lowcmd`.

Run with launch:
```bash
source /home/sunxd/robustCapture/unitree_ws/unitree_ros_ws/devel/setup.bash
roslaunch z1_sdk_ros z1_lowcmd_pub.launch
```

Or quick CLI test (single publish):
```bash
rostopic pub -1 /z1/lowcmd unitree_z1_msgs/LowCmd \"{q:[0,0,0,0,0,0,0], qd:[0,0,0,0,0,0,0], tau:[0,0,0,0,0,0,0], kp:[20,30,30,20,15,10,20], kd:[2000,2000,2000,2000,2000,2000,2000]}\"
```

## Panda XY Tracking Controller
This node tracks the Panda end-effector **X/Y** in the `world` frame and keeps a fixed Z offset relative to the Z1 base.
It publishes `/z1/lowcmd` using the Z1 SDK kinematics for IK and inverse dynamics.

Run:
```bash
source /home/sunxd/robustCapture/unitree_ws/unitree_ros_ws/devel/setup.bash
roslaunch z1_sdk_ros z1_panda_xy_controller.launch
```

Key params:
- `~world_frame` (default `world`)
- `~panda_ee_frame` (default `panda_link8`)
- `~z1_base_frame` (default `z1_base`)
- `~z_offset` (meters)
- `~rate_hz` (default `200`)
- `~tau_limit` (default `30.0`)
- `~enable_gripper` (default `true`)
- `~q_init` (6‑element IK seed, optional)
- `~state_topic` (default `/z1/joint_states`)
- `~use_state` (default `true`, use Z1 joint state as IK seed)

Note: this controller relies on TF being available for the Panda end effector and the Z1 base in the `world` frame.

## Z1 Base Static TF (world -> z1_base)
If you don't have a dynamic TF for the Z1 base, you can publish a fixed transform:
```bash
roslaunch z1_sdk_ros z1_static_tf.launch
```
Edit the transform in `unitree_ws/unitree_ros_ws/src/z1_sdk_ros/launch/z1_static_tf.launch` to match your setup.
The 6 numbers in the `args` are:
- `X Y Z` = translation in meters
- `R P Y` = rotation in radians (roll, pitch, yaw)

## Panda Base Static TF (world -> panda_link0)
To align the `world` frame with the Panda base frame:
```bash
roslaunch z1_sdk_ros panda_world_tf.launch
```
This launch file sets `world` and `panda_link0` to the same pose.

## Franka robot_state_publisher
The Franka stack already starts `robot_state_publisher` in these launch files:
- `franka_ws/franka_ros_ws/src/franka_ros/franka_control/launch/franka_control.launch`
- `franka_ws/franka_ros_ws/src/franka_ros/franka_control/launch/franka_combined_control.launch`
- `franka_ws/franka_ros_ws/src/franka_ros/franka_gazebo/launch/robot.launch`
- `franka_ws/franka_ros_ws/src/franka_ros/franka_visualization/launch/franka_visualization.launch`

If you use a custom Franka launch file, make sure it includes the `robot_state_publisher` node.

The example launch `franka_ws/franka_ros_ws/src/franka_ros/franka_example_controllers/launch/cartesian_impedance_example_controller.launch`
includes `franka_control.launch` via:
```xml
<include file="$(find franka_control)/launch/franka_control.launch" pass_all_args="true"/>
```
so it also starts `robot_state_publisher`.

## Topic Definition
The ROS topic is defined in two places:
- Launch file (explicit): `unitree_ws/unitree_ros_ws/src/z1_sdk_ros/launch/z1_lowcmd.launch` with `<param name="topic" value="/z1/lowcmd"/>`.
- Node default (fallback): `unitree_ws/unitree_ros_ws/src/z1_sdk_ros/src/z1_lowcmd_node.cpp` uses `pnh.param("topic", topic, "/z1/lowcmd")`.

You can override at launch time:
```bash
roslaunch z1_sdk_ros z1_lowcmd.launch topic:=/my_topic
```

## Topic Name vs Message Type
- `/z1/lowcmd` is the **topic name** (just a string on the ROS graph).
- `unitree_z1_msgs/LowCmd` is the **message type** (the data structure on that topic).

You can publish `unitree_z1_msgs/LowCmd` on any topic name you want, as long as publisher and subscriber match.

## Launch Files and Nodes
A launch file can start **one or many nodes**. It can also set parameters, remap topics, and include other launch files.  
In this workspace, `unitree_ws/unitree_ros_ws/src/z1_sdk_ros/launch/z1_lowcmd.launch` starts a single node: `z1_lowcmd_node`.

## Topics Don’t Need Folders
A ROS topic is just a **name** on the ROS graph. You can set or override it in code, a launch file, or the CLI.  
You do **not** create folders for topics. Folders only exist for packages (code, messages, launch files).

## Topic vs Node (Graph Intuition)
- **Node** = a running process (executable).
- **Topic** = a named data stream used for communication.

Graph mental model:
- Node = vertex
- Topic = edge (many publishers and many subscribers can connect to the same topic)

## One Laptop vs Separate Robot PCs (Networking)
ROS does **not** map “ports to robots.” It routes by node/topic names, and drivers use IPs to talk to hardware.

**Case A: One laptop controls both robots (your setup)**  
- All ROS nodes run on the same laptop.
- Each robot is reached by its **own IP address** via its driver/SDK.
- Separation is done by **topic namespaces** (e.g., `/panda/...`, `/z1/...`) and node names.

**Case B: Each robot has its own controller PC (general setup)**  
- Each PC runs its robot’s ROS nodes.
- All PCs point to the same ROS master (or use ROS multi‑master).
- You must set `ROS_MASTER_URI` and `ROS_IP` on each machine.

In both cases, drivers use the robot IP; ROS just connects nodes by topic name.

### Parameters
- `~topic` (default `/z1/lowcmd`)
- `~has_gripper` (default `true`)
- `~cmd_timeout` (default `0.2` seconds)
- `~set_lowcmd_fsm` (default `true`)
- `~rate_hz` (default `0` = use SDK dt)
