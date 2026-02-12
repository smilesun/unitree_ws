cmake_minimum_required(VERSION 3.0.2)
project(z1_panda_bridge)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(panda_state_relay src/panda_state_relay.cpp)
target_link_libraries(panda_state_relay ${catkin_LIBRARIES})

install(TARGETS panda_state_relay
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
