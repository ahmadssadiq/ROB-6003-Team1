cmake_minimum_required(VERSION 3.10)
project(roomrunner_interfaces)

# Generate the message code with the Python ROS was built with, not the conda environment's Python
# (same reason as in aruco_interfaces).
set(Python3_EXECUTABLE /usr/bin/python3)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjectPose.msg"
  "msg/ObjectPoseArray.msg"
  "msg/TaskStep.msg"
  "msg/TaskSpec.msg"
  "msg/MissionStatus.msg"
  "srv/PlanPath.srv"
  "action/NavigateTo.action"
  "action/PickPlace.action"
  DEPENDENCIES geometry_msgs nav_msgs std_msgs
)

ament_package()
