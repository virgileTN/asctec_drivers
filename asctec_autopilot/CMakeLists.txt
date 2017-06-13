cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories (${CMAKE_CURRENT_BINARY_DIR})

# create autopilot library

rosbuild_add_library (autopilot src/autopilot.cpp
                                src/serial_interface.cpp
                                src/crc16.cpp 
                                src/telemetry.cpp)

# create autopilot_nodelet library

rosbuild_add_library (autopilot_nodelet src/autopilot_nodelet.cpp)
target_link_libraries (autopilot_nodelet autopilot)

# create autopilot_node executable

rosbuild_add_executable(autopilot_node src/autopilot_node.cpp)
target_link_libraries (autopilot_node autopilot)

#rosbuild_add_executable(takeoff src/takeoff.cpp)
#rosbuild_add_executable(waypoint_test src/waypoint_test.cpp)
#rosbuild_add_executable(land src/land.cpp)
#rosbuild_add_executable(envoi_wp_simple src/envoi_wp_simple.cpp)
#rosbuild_add_executable(circular_route src/circular_route.cpp)
#rosbuild_add_executable(batterie src/batterie.cpp)

#rosbuild_add_executable(test_node src/test_node.cpp)
#rosbuild_add_executable(test_node2 src/test_node2.cpp)

#rosbuild_add_executable(formation src/drone_formation.cpp)
#rosbuild_add_executable(formation2 src/drone_formation_sans_wp_inter.cpp)
#rosbuild_add_executable(formation_4 src/drone_formation4_sans_wp_inter.cpp)
#rosbuild_add_executable(flocking src/flocking.cpp)
#rosbuild_add_executable(fake_flocking src/fake_flocking.cpp)
#rosbuild_add_executable(new_flocking src/new_flocking.cpp)
rosbuild_add_executable(fdp src/fdp.cpp)
rosbuild_add_executable(very_new_flocking src/very_new_flocking.cpp)
#rosbuild_add_executable(trace_gps_simple src/trace_gps_simple.cpp)
#rosbuild_add_executable(boite_noire src/boite_noire.cpp)
#rosbuild_add_executable(batterie src/batterie.cpp)
