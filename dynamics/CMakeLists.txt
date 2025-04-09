cmake_minimum_required(VERSION 3.5)
project(dynamics)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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

set(PINOCCHIO_FLAGS
  ${pinocchio_CFLAGS_OTHER}
  -Wno-ignored-attributes
  -Wno-invalid-partial-specialization    # to silence warning with unsupported Eigen Tensor
  -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
  -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(utils REQUIRED)
find_package(trajectories REQUIRED)
find_package(customized_msgs REQUIRED)
find_package(system REQUIRED)

# temporary fix for customized_msgs include dir
# message(FATAL_ERROR "customized_msgs: ${msgs_INCLUDE_DIRS}")
include_directories(/workspaces/kinova_robust_control_docker/install/customized_msgs/include)

# urdf support
find_package(urdf)
add_definitions(-DURDF_SUPPORT)
find_package(ament_cmake_python REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})

# Find Pinocchio library
find_package(pinocchio REQUIRED)
include_directories(${PINOCCHIO_INCLUDE_DIRS})
link_directories(${PINOCCHIO_LIBRARY_DIRS})

# CONAN {{
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
include(${PROJECT_SOURCE_DIR}/../thirdParty/conan.cmake)

conan_check(REQUIRED)
conan_cmake_run(
  REQUIRES tinyxml2/9.0.0
  BASIC_SETUP
  UPDATE
  NO_OUTPUT_DIRS # NOTE: deleting this will change ros default binary output path
  )

link_libraries(${CONAN_LIBS})
#}}

include_directories(include)

# BUILDING {{
add_library(
  ${PROJECT_NAME}
  src/dynamics/Spatial.cpp
  src/dynamics/Model.cpp
  src/dynamics/RNEA.cpp
  src/dynamics/ControllerBlock.cpp
  src/dynamics/PIDControlBlock.cpp
  src/dynamics/GravityCompensationPIDControlBlock.cpp
  src/dynamics/PassivityControlBlock.cpp
  src/dynamics/ARMOURControlBlock.cpp
  )
target_link_libraries(${PROJECT_NAME}
  pinocchio::pinocchio)
ament_target_dependencies(${PROJECT_NAME}
  utils
  system
  trajectories
  customized_msgs
  urdf
  Eigen3)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  utils
  system
  trajectories
  customized_msgs
  urdf
  Eigen3
)
target_compile_options(${PROJECT_NAME} PUBLIC
  ${PINOCCHIO_FLAGS})

# Install {{
link_libraries(${PROJECT_NAME})
# export include dir
install(
  DIRECTORY include/
  DESTINATION include
  )
# export lib
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
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
    rnea_test
    test/TestInverseDynamics.cpp
    )
  target_link_libraries(
    rnea_test
    GTest::gtest_main
    Eigen3::Eigen
    ${PKG_PIN_CONFIG_LIBRARIES}
    )

  gtest_discover_tests(rnea_test)
  # install executable
  install(TARGETS
    rnea_test
    DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
