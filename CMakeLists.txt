cmake_minimum_required(VERSION 2.8.3)
project(control_demo)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    genmsg
    actionlib_msgs
    actionlib
    moveit_msgs
    geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

add_action_files(DIRECTORY action FILES main.action)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  actionlib_msgs
)

catkin_package(
  LIBRARIES
    interactivity_utils
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)


###########
## Build ##
###########

include_directories(${THIS_PACKAGE_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})




add_executable(franka_control src/franka_control.cpp)
target_link_libraries(franka_control ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS franka_control DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# add_executable(plan_client src/plan_client.cpp)
# target_link_libraries(plan_client ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# install(TARGETS plan_client DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})