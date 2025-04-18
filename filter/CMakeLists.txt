cmake_minimum_required(VERSION 3.5)
project(filter)
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

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)
# a simple place holder, just so we can pass the compilation
add_library(${PROJECT_NAME} src/nothing.cpp)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
# ament_target_dependencies(${PROJECT_NAME} system)

# export include dir
install(
  DIRECTORY include/
  DESTINATION include
  )

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
  find_package(system)
  include(GoogleTest)

  # Add a gtest
  enable_testing()

  add_executable(
    filter_test
    test/filter_test.cpp
    )
  target_link_libraries(
    filter_test
    GTest::gtest_main
    )
  ament_target_dependencies(
    filter_test
    system
    )

  gtest_discover_tests(filter_test)

  install(TARGETS
    filter_test
    DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
