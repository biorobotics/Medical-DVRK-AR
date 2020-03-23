cmake_minimum_required(VERSION 2.8.3)
project(blaser_pcl)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslaunch
  std_msgs
  sensor_msgs
  geometry_msgs
  image_transport
  cv_bridge
  tf2
  tf2_ros
  pcl_ros
  pcl_conversions
  message_generation
  dynamic_reconfigure
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
find_package(OpenCV 3 REQUIRED)
# find_package(Eigen3 REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/LaserExtraction.cfg
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

# Generate services in the 'srv' folder
add_service_files(
  FILES
  VoxelGridStitch.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  # ${EIGEN3_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIB_DIR}
)

add_definitions(
    ${PCL_DEFINITIONS}
)

add_executable(blaser_image_pub_node src/blaser_image_pub.cpp)
target_link_libraries(blaser_image_pub_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(blaser_pointcloud_pub_node src/blaser_pointcloud_pub.cpp src/pclGen.cpp src/im_proc.cpp)
target_link_libraries(blaser_pointcloud_pub_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(blaser_pointcloud_pub_node ${PROJECT_NAME}_gencfg)

add_executable(voxel_grid_filter_node src/voxel_grid_filter.cpp)
target_link_libraries(voxel_grid_filter_node ${catkin_LIBRARIES})

add_executable(pointcloud_saver_node src/pointcloud_saver.cpp)
target_link_libraries(pointcloud_saver_node ${catkin_LIBRARIES})

add_executable(testim_node src/testim.cpp src/pclGen.cpp src/im_proc.cpp)
target_link_libraries(testim_node ${catkin_LIBRARIES} ${OpenCV_LIBS})


