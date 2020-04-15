cmake_minimum_required(VERSION 2.8.3)
project(task_control)

find_package(catkin REQUIRED COMPONENTS
  objdt
  apriltags2_ros
  realsense2_camera
  roscpp
  rospy
  geometry_msgs
  std_msgs
)

catkin_python_setup()

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS apriltags2_ros realsense2_camera roscpp rospy geometry_msgs std_msgs
)

catkin_install_python(
  PROGRAMS nodes/simple_control
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
