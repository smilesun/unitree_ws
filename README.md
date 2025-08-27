# Unitree Workspace (with Git Submodules)

This repository uses Git submodules to include several dependent repositories.

## Submodules

unitree_ros_ws/src/unitree_ros → your fork of unitree_ros

z1_controller → your fork of z1_controller

z1_sdk → your fork of z1_sdk

## How to clone with submodules

```bash
git clone --recurse-submodules git@github.com:prominentjohnson/unitree_ws.git
```

If you already cloned without submodules

```bash
cd unitree_ws
git submodule update --init --recursive
```

## How to run ROS simulation

### Prerequest

The simulation can be run on ROS melodic or neotic. Therefore, you should install Ubuntu 18.04 or Ubuntu 20.04 respectively. Make sure ROS is installed.

### Set ROS workspace

```bash
cd unitree_ros_ws
catkin_make
source devel/setup.bash
```

### Run launch file

Run
```bash
roslaunch unitree_gazebo z1.launch UnitreeGripperYN:=true
```
to start simulation with the gripper. Set UnitreeGripperYN to false to run simulation without gripper.

If successfully configured, the simulation interface of Gazebo will be displayed.

## How to Enable Control using z1_controller

In some of the old document provided by Unitree, they switch between
```bash
set(COMMUNICATION UDP)             #UDP
set(COMMUNICATION ROS)             #ROS
```
in the CMakeLists.txt to enable control for the real machine or simulation. However this is not available in the current code because there's no such lines in the CMakeLists.txt.

To control the z1 robot, the first step is to build the targets:
```bash
cd z1_controller
mkdir build
cd build
cmake ..
make
```
Then if you want to control the robot in the simulation, execute `./sim_ctrl`.
Instead, if you want to control the real machine, execute `./z1_ctrl`.
If you want to use keyboard control, execute`./z1_ctrl k`for the real machine, and`./sim_ctrl k` for the simulation.

## How to use z1_sdk

First, build the targets:
```bash
cd z1_sdk
mkdir build
cd build
cmake ..
make
```
In the simulation, after executing
```bash
./sim_ctrl
```
z1_controller tries to communicate with z1_sdk. Now by executing
```bash
./highcmd_basic
```
The robot will start to perform a demo(dance) in the simulation.
Simmilarly, if you want the robot to dance in the real world. Combine `./z1_ctrl` and `./highcmd_basic`.







