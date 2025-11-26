cmake_minimum_required(VERSION 3.8)
project(locomotion_msgs)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(rosidl_default_generators REQUIRED)

set(dependencies
  std_msgs
  geometry_msgs
)

set(msg_files
  "msg/RobotStates.msg"
  "msg/RigidBody.msg"
  "msg/PlanCommands.msg"
)

set(srv_files
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES ${dependencies}
)

ament_auto_package()
