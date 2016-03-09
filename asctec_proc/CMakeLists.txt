cmake_minimum_required(VERSION 2.8.3)
project(asctec_proc)

find_package(catkin REQUIRED COMPONENTS
  asctec_msgs
  mav_msgs
  nav_msgs
  nodelet
  roscpp
  sensor_msgs
  std_msgs
  tf
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES asctec_proc asctec_proc_nodelet
  CATKIN_DEPENDS
    asctec_msgs
    mav_msgs
    nodelet
    roscpp
    sensor_msgs
    std_msgs
    tf
  )

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(asctec_proc_nodelet
  src/asctec_proc_nodelet.cpp
  )
target_link_libraries(asctec_proc_nodelet asctec_proc ${catkin_LIBRARIES})
add_dependencies(asctec_proc_nodelet ${catkin_EXPORTED_TARGETS})

add_library(asctec_proc
  src/asctec_proc.cpp
  )
target_link_libraries(asctec_proc ${catkin_LIBRARIES})
add_dependencies(asctec_proc ${catkin_EXPORTED_TARGETS})

add_executable(asctec_proc_node
  src/asctec_proc_node.cpp
  )
target_link_libraries(asctec_proc_node asctec_proc ${catkin_LIBRARIES})

install(TARGETS asctec_proc_node asctec_proc_nodelet asctec_proc
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(FILES asctec_proc_nodelet.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
