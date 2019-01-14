cmake_minimum_required(VERSION 2.8.3)
project(sensor_data_transport_wrapper)

add_compile_options(-std=c++11
                    -O3
#                    -march=native
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  image_transport
  cv_bridge
  sensor_msgs
  pcl_ros
  tf2
  tf2_ros
  realsense_camera_wrapper
)

find_package(PCL 1.7 REQUIRED)
find_package(realsense2 REQUIRED COMPONENTS)
find_package(OpenCV REQUIRED COMPONENTS)
find_package(Boost REQUIRED COMPONENTS)

# include realsence_camera_wraper library
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    realsense_camera_wrapper
  CATKIN_DEPENDS
    realsense_camera_wrapper
)
## Specify additional locations of header files
include_directories(
  include/realsense_camera_wrapper
  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(pointcloud_transport_wrapper_node_test
  src/pointcloud_transport_wrapper_node_test.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(pointcloud_transport_wrapper_node_test
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  zmq
)

add_executable(camera_ros_zmq_sub_example
    src/camera_ros_zmq_sub_example.cpp
)

target_link_libraries(camera_ros_zmq_sub_example
    zmq
    ${PCL_LIBRARIES}
)

add_executable(pointcloud_transport_wrapper
    src/pointcloud_transport_wrapper.cpp
)

target_link_libraries(pointcloud_transport_wrapper
    zmq
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

