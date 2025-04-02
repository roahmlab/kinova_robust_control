cmake_minimum_required(VERSION 3.5)
project(roahm_experiments)

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
find_package(ament_cmake_python REQUIRED)
find_package(roahm_msgs REQUIRED)
find_package(roahm_trajectories REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
#
ament_python_install_package(${PROJECT_NAME})
install(
  PROGRAMS
  scripts/test_services.py
  scripts/pick_up_example.py
  scripts/trajectory_helper.py
  scripts/test_trajectories.py
  scripts/test_trajectories_dual_arm.py
  scripts/test_armtd_trajectories.py
  scripts/test_multiple_continuous_trajectories.py
  scripts/test_receding_horizon_trajectories.py
  scripts/test_receding_horizon_trajectories_armtd.py
  scripts/parse_rosbag.py
  scripts/pinocchio_ik_solver.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
