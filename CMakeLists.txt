cmake_minimum_required(VERSION 3.0.2)
project(object_detection)

find_package(catkin REQUIRED COMPONENTS
  rosbag
  roscpp
  tf2_eigen
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)


catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(object_detection src/object_detection.cpp src/imageCapture.cpp src/imageProcess.cpp src/shapeHandling.cpp)

add_dependencies(object_detection ${object_detection_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(object_detection
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

