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
