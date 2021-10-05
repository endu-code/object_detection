cmake_minimum_required(VERSION 3.1.3)
project(endu_robotics)

find_package(catkin REQUIRED
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_move_group
  moveit_visual_tools
  moveit_core
  interactive_markers
  moveit_ros_perception
  pluginlib
  pcl_ros
  pcl_conversions
  rosbag
  roscpp
  std_msgs
  tf2_eigen
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)


catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(braccio_test_manipulation src/braccio_test_manipulation.cpp)
add_executable(parse_publish src/parse_and_publish.cpp)

add_dependencies(braccio_test_manipulation ${braccio_test_manipulation_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(parse_publish ${parse_and_publish_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(braccio_test_manipulation
  ${catkin_LIBRARIES}
)
target_link_libraries(parse_publish
  ${catkin_LIBRARIES}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
