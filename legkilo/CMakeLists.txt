cmake_minimum_required(VERSION 3.5)
project(legkilo)

set(CMAKE_BUILD_TYPE "Release") 

message(INFO "CMake build type:  ${CMAKE_BUILD_TYPE}")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions  -ggdb ")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS}")

find_program(SCCACHE_PROGRAM sccache)
if(SCCACHE_PROGRAM)
  set(CMAKE_CXX_COMPILER_LAUNCHER "${SCCACHE_PROGRAM}")
  message(STATUS "Using sccache: ${SCCACHE_PROGRAM}")
else()
  find_program(CCACHE_PROGRAM ccache)
  if(CCACHE_PROGRAM)
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    message(STATUS "Using ccache: ${CCACHE_PROGRAM}")
  endif()
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

message(INFO "Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
include(ProcessorCount)
ProcessorCount(CPU_N)
message(INFO "Processer number:  ${CPU_N}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Glog REQUIRED)
find_package(go2_driver REQUIRED)
# find_package(GTSAM REQUIRED)


set(dependencies
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  pcl_conversions
  tf2
  tf2_ros
  tf2_geometry_msgs
  go2_driver
)

set(SOURCES
    # Preprocessing modules
    src/preprocess/lidar_processing.cc
    src/preprocess/kinematics.cc

    # Lidar Preprocessing
    src/preprocess/lidar/hesai.cc
    src/preprocess/lidar/ouster.cc
    src/preprocess/lidar/robosense.cc
    src/preprocess/lidar/velodyne.cc
    src/preprocess/lidar/utlidar.cc
    
    # Core algorithms
    src/core/slam/KILO.cc
    src/core/slam/voxel_map.cc
    src/core/slam/eskf.cc
    
    # ROS interface
    src/interface/ros2/ros_interface.cc
    src/interface/ros2/options.cc
)

add_library(${PROJECT_NAME} ${SOURCES})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${yaml-cpp_INCLUDE_DIRS}
    ${Glog_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${PCL_FILTERS_LIBRARIES}
    yaml-cpp
    glog
    gflags
)

add_executable(${PROJECT_NAME}_node
    src/apps/leg_kilo_node.cc
)
ament_target_dependencies(${PROJECT_NAME}_node ${dependencies})
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}  
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  rviz
  robot_urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
