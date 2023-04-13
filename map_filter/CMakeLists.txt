cmake_minimum_required(VERSION 3.8)
project(map_filter)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)	
find_package(rclcpp REQUIRED)#
find_package(pcl_ros REQUIRED)#
find_package(pcl_conversions REQUIRED)#
find_package(sensor_msgs REQUIRED)#
find_package(geometry_msgs REQUIRED)#

## pcl install
cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(map_filter src/map_filter.cpp)
target_link_libraries(map_filter ${PCL_LIBRARIES})
target_link_libraries(map_filter
  ${Boost_SYSTEM_LIBRARY}  # pcl_conversions requires this
)

ament_target_dependencies(
  map_filter
  geometry_msgs #
  pcl_conversions#
  rcl_interfaces#
  pcl_ros#
  rclcpp #
  rcutils
  sensor_msgs#
  std_msgs
)

install(TARGETS
  map_filter
  DESTINATION lib/${PROJECT_NAME}/
)
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

ament_export_dependencies(libpcl-all-dev pcl_ros pcl_conversions sensor_msgs)

ament_package()
