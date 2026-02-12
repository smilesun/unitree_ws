What is a ROS message?

  - A ROS message is a typed data structure used for ROS topics/services.
  -   - It’s defined in a plain text .msg file, then ROS generates C++/Python code from it.
-   How it exists in a workspace
  - It lives inside a ROS package (a folder with a package.xml).
  -   - Typical layout:
    -       - unitree_z1_msgs/package.xml
    -       - unitree_z1_msgs/msg/LowCmd.msg
    -       - unitree_z1_msgs/CMakeLists.txt
    -   - After catkin_make, ROS generates headers so you can #include "unitree_z1_msgs/LowCmd.h"
      and use it in C++ nodes
    - 
# Example
## Z1 low-level command. Units follow SDK conventions.
### 6 arm joints + 1 gripper (last element for gripper).
float64[7] q    # joint position command (rad), gripper in q[6]
float64[7] qd   # joint velocity command (rad/s), gripper in qd[6]
float64[7] tau  # joint torque feedforward (Nm), gripper in tau[6]
float64[7] kp   # joint position gains
float64[7] kd   # joint velocity gains

