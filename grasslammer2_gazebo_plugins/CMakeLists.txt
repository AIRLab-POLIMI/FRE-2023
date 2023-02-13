cmake_minimum_required(VERSION 3.8)
project(grasslammer2_gazebo_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)

add_library(grasslammer2_gazebo_differential_plugin SHARED
  src/differential_plugin.cpp
)

ament_target_dependencies(grasslammer2_gazebo_differential_plugin
  gazebo_dev
  gazebo_ros
)
ament_export_libraries(grasslammer2_gazebo_differential_plugin)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(
  TARGETS grasslammer2_gazebo_differential_plugin
  LIBRARY DESTINATION lib
)
