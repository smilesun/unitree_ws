cmake_minimum_required(VERSION 3.0.2)
project(dual_arm_mpc)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  unitree_z1_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/dual_arm_mpc_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
