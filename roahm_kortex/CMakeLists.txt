cmake_minimum_required(VERSION 3.5)
project(roahm_kortex)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Release)

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
find_package(roahm_utils REQUIRED)
find_package(roahm_trajectories REQUIRED)
find_package(roahm_msgs REQUIRED)
find_package(roahm_system REQUIRED)
find_package(roahm_dynamics REQUIRED)
find_package(roahm_filter)
find_package(rclcpp REQUIRED)

# temporary fix for roahm_msgs include dir
# message(FATAL_ERROR "roahm_msgs: ${roahm_msgs_INCLUDE_DIRS}")
include_directories(/workspaces/kinova_robust_control_docker/install/roahm_msgs/include)

# Find Pinocchio library
find_package(pinocchio REQUIRED)
include_directories(${PINOCCHIO_INCLUDE_DIRS})
link_directories(${PINOCCHIO_LIBRARY_DIRS})

set(PINOCCHIO_FLAGS
  ${pinocchio_CFLAGS_OTHER}
  -Wno-ignored-attributes
  -Wno-invalid-partial-specialization    # to silence warning with unsupported Eigen Tensor
  -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
  -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)

# CONAN {{
include(${PROJECT_SOURCE_DIR}/../thirdParty/conan.cmake)

conan_check(REQUIRED)
conan_add_remote(
  NAME kinova_public
  URL https://artifactory.kinovaapps.com/artifactory/api/conan/conan-public
)

conan_cmake_run(
  REQUIRES kortex_api_cpp/2.3.0-r.34@kortex/stable
  SETTINGS kortex_api_cpp:compiler=gcc
  SETTINGS kortex_api_cpp:compiler.version=5
  SETTINGS compiler.libcxx=libstdc++11
  PROFILE_AUTO build_type
  BASIC_SETUP
  UPDATE
  NO_OUTPUT_DIRS # NOTE: deleting this will change ros default binary output path
)

link_libraries(${CONAN_LIBS})
# }}


# BUILD {{
include_directories(include)
set(SRC_FILES
  src/roahm_kortex/KortexRobot.cpp
  src/roahm_kortex/KortexBlock.cpp
  src/roahm_kortex/JointFilterBlock.cpp
)

# add_executable(rnea_system src/rnea_system.cpp ${SRC_FILES})
# ament_target_dependencies(rnea_system
#   roahm_dynamics
#   roahm_filter
# )

# add_executable(robust_control_system src/robust_control_system.cpp ${SRC_FILES})
# ament_target_dependencies(robust_control_system
#   roahm_dynamics
#   roahm_filter
# )

add_executable(ros_kortex_system src/ros_kortex_system.cpp ${SRC_FILES})
target_link_libraries(ros_kortex_system
  fmt::fmt)
ament_target_dependencies(ros_kortex_system
  rclcpp
  roahm_dynamics
  roahm_filter
)

add_executable(control_system 
  src/control_system.cpp 
  ${SRC_FILES})
target_link_libraries(control_system
  pinocchio::pinocchio
  fmt::fmt)
ament_target_dependencies(control_system
  roahm_dynamics
  roahm_filter
)
target_compile_options(control_system PUBLIC
  ${PINOCCHIO_FLAGS})

install(TARGETS 
  control_system
  ros_kortex_system
  DESTINATION lib/${PROJECT_NAME})

install(
  PROGRAMS test/cartesian_control_test.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
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

  # add gtest
  include(FetchContent)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG release-1.12.1
    )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  # Include linting tests
  find_package(ament_lint_auto REQUIRED)
  include(GoogleTest)

  # Add a gtest
  enable_testing()

  add_executable(
    kortex_test
    test/kortex_test.cpp
    ${SRC_FILES}
    )
  target_link_libraries(
    kortex_test
    GTest::gtest_main
    fmt::fmt
    )
  ament_target_dependencies(
    kortex_test
    roahm_utils
    roahm_trajectories
    roahm_msgs
    roahm_filter
    roahm_system
    )

  gtest_discover_tests(kortex_test)

  install(TARGETS
    kortex_test
    DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
