cmake_minimum_required(VERSION 3.0.2)
project(z1_sdk_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  unitree_z1_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Locate Z1 SDK relative to this package.
get_filename_component(Z1_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../z1_sdk" ABSOLUTE)
if (NOT EXISTS "${Z1_SDK_ROOT}/include" OR NOT EXISTS "${Z1_SDK_ROOT}/lib")
  message(FATAL_ERROR "Z1 SDK not found at ${Z1_SDK_ROOT}. Expected include/ and lib/.")
endif()

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs unitree_z1_msgs tf2 tf2_ros tf2_geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Z1_SDK_ROOT}/include
)

link_directories(${Z1_SDK_ROOT}/lib)

add_executable(z1_lowcmd_node src/z1_lowcmd_node.cpp)
add_dependencies(z1_lowcmd_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(z1_lowcmd_node ${catkin_LIBRARIES} Z1_SDK_${CMAKE_SYSTEM_PROCESSOR})

add_executable(z1_lowcmd_pub src/z1_lowcmd_pub.cpp)
add_dependencies(z1_lowcmd_pub ${catkin_EXPORTED_TARGETS})
target_link_libraries(z1_lowcmd_pub ${catkin_LIBRARIES})

add_executable(z1_panda_xy_controller src/z1_panda_xy_controller.cpp)
add_dependencies(z1_panda_xy_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(z1_panda_xy_controller ${catkin_LIBRARIES} Z1_SDK_${CMAKE_SYSTEM_PROCESSOR})

install(TARGETS z1_lowcmd_node z1_lowcmd_pub z1_panda_xy_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
