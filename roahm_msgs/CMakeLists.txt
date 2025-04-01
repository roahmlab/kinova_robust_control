cmake_minimum_required(VERSION 3.5)
project(roahm_msgs)
set(CMAKE_EXPORT_COMPILE_COMMANDS On)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(roahm_trajectories REQUIRED)
find_package(std_msgs REQUIRED)

# generate message
set(msg_files
  "msg/TorqueControl.msg"
  "msg/KortexMeasurements.msg"
  "msg/TrajectoryMsg.msg"
  )

# generate service files
set(srv_files
  "srv/GotoName.srv"
  "srv/Goto.srv"
  "srv/RefreshJoints.srv"
  "srv/GotoCartesian.srv"
  "srv/GripperAction.srv"
  "srv/Bool.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs
)
ament_export_dependencies(
  rosidl_default_runtime
  std_msgs
  roahm_utils
  roahm_trajectories)

# installs
install(
  DIRECTORY include/
  DESTINATION include
  )

ament_package()
