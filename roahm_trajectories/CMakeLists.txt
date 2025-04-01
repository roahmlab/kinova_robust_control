cmake_minimum_required(VERSION 3.5)
project(roahm_trajectories)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(roahm_utils REQUIRED)
find_package(roahm_system REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(fmt REQUIRED)

# CONAN {{
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
include(${PROJECT_SOURCE_DIR}/../thirdParty/conan.cmake)

conan_check(REQUIRED)
conan_cmake_run(
  BASIC_SETUP
  UPDATE
  NO_OUTPUT_DIRS # NOTE: deleting this will change ros default binary output path
  )

link_libraries(${CONAN_LIBS})
#}}

include_directories(include)

add_library(
  ${PROJECT_NAME}
  src/roahm_trajectories/Trajectories.cpp
  src/roahm_trajectories/TrajectoryManager.cpp
  )
ament_target_dependencies(${PROJECT_NAME}
  roahm_utils
  roahm_system
  Eigen3
  )
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  roahm_utils
  roahm_system
  Eigen3
  )

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

# pybind program
pybind11_add_module(roahm_trajectories_py MODULE 
  src/roahm_trajectories_pybind.cpp
  src/roahm_trajectories/TrajectoryPybindWrapper.cpp
  src/roahm_trajectories/Trajectories.cpp
  )
target_link_libraries(roahm_trajectories_py PRIVATE 
  fmt::fmt)
ament_target_dependencies(roahm_trajectories_py
  PUBLIC
  pybind11_vendor
  roahm_utils
  roahm_system
  Eigen3
)
set(PYTHON_INSTALL_DIR "lib/python3.10/site-packages")
install(
  TARGETS roahm_trajectories_py
  DESTINATION "${PYTHON_INSTALL_DIR}"
)

ament_python_install_package(${PROJECT_NAME})
install(
  PROGRAMS
  roahm_trajectories/TrajectoryMacros.py
  DESTINATION lib/${PROJECT_NAME}
)

# unit tests
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
ament_lint_auto_find_test_dependencies()

# Add a gtest
enable_testing()

add_executable(
  armour_trajectory_test
  test/TestArmourBezierCurve.cpp
  )
target_link_libraries(
  armour_trajectory_test
  GTest::gtest_main
  Eigen3::Eigen
  )
ament_target_dependencies(
  armour_trajectory_test
  roahm_utils
  roahm_system
  Eigen3
  )
gtest_discover_tests(armour_trajectory_test)
install(TARGETS
  armour_trajectory_test
  DESTINATION lib/${PROJECT_NAME})

add_executable(
  Bezier_trajectory_test
  test/TestBezierCurve.cpp
  )
target_link_libraries(
  Bezier_trajectory_test
  GTest::gtest_main
  Eigen3::Eigen
  )
ament_target_dependencies(
  Bezier_trajectory_test
  roahm_utils
  roahm_system
  Eigen3
  )
gtest_discover_tests(Bezier_trajectory_test)
install(TARGETS
  Bezier_trajectory_test
  DESTINATION lib/${PROJECT_NAME})

add_executable(
  Fourier_trajectory_test
  test/TestFixedFrequencyFourierCurve.cpp
  )
target_link_libraries(
  Fourier_trajectory_test
  GTest::gtest_main
  Eigen3::Eigen
  )
ament_target_dependencies(
  Fourier_trajectory_test
  roahm_utils
  roahm_system
  Eigen3
  )
gtest_discover_tests(Fourier_trajectory_test)
install(TARGETS
  Fourier_trajectory_test
  DESTINATION lib/${PROJECT_NAME})

add_executable(
  trajectory_manager_test
  test/TestTrajectoryManager.cpp
  )
target_link_libraries(
  trajectory_manager_test
  GTest::gtest_main
  Eigen3::Eigen
  fmt::fmt
  )
ament_target_dependencies(
  trajectory_manager_test
  roahm_utils
  roahm_system
  Eigen3
  )
install(TARGETS
  trajectory_manager_test
  DESTINATION lib/${PROJECT_NAME})

ament_package()
