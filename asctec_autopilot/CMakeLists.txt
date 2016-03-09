cmake_minimum_required(VERSION 2.8.3)
project(asctec_autopilot)

find_package(catkin REQUIRED COMPONENTS
  asctec_msgs
  diagnostic_updater
  geometry_msgs
  nav_msgs
  nodelet
  sensor_msgs
  roscpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES autopilot autopilot_nodelet
  CATKIN_DEPENDS
    asctec_msgs
    diagnostic_updater
    nodelet
    roscpp
  )

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(autopilot_nodelet
  src/autopilot_nodelet.cpp
  )
target_link_libraries(autopilot_nodelet autopilot ${catkin_LIBRARIES})
add_dependencies(autopilot_nodelet ${catkin_EXPORTED_TARGETS})

add_library(autopilot
  src/autopilot.cpp
  src/serial_interface.cpp
  src/crc16.cpp
  src/telemetry.cpp
  )
target_link_libraries(autopilot ${catkin_LIBRARIES})
add_dependencies(autopilot ${catkin_EXPORTED_TARGETS})

add_executable(autopilot_node
  src/autopilot_node.cpp
  )
target_link_libraries(autopilot_node autopilot ${catkin_LIBRARIES})

install(TARGETS autopilot autopilot_nodelet autopilot_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(FILES autopilot_nodelet.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
