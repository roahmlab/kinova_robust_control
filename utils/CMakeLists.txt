cmake_minimum_required(VERSION 3.5)
project(utils)

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

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(spdlog REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(fmt REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Build{{
include_directories(include)
add_library(${PROJECT_NAME}
  src/utils/Utils.cpp)
ament_target_dependencies(${PROJECT_NAME} Eigen3)
target_link_libraries(${PROJECT_NAME} spdlog)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(spdlog)
# }}

# Install {{
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
# }}

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  link_libraries(${PROJECT_NAME})
  add_executable(logger_test test/logger_test.cpp)
  target_link_libraries(logger_test PRIVATE fmt::fmt)
  add_executable(signal_test test/signal_test.cpp)
  ament_target_dependencies(signal_test rclcpp)
  install(TARGETS
    logger_test
    signal_test
    DESTINATION lib/${PROJECT_NAME}
    )
endif()

ament_package()
