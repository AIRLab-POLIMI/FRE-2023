cmake_minimum_required(VERSION 3.8)
project(ground_removal)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-dev)
endif()


find_package(ament_cmake REQUIRED)	
#find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)#
find_package(pcl_ros REQUIRED)#
find_package(pcl_conversions REQUIRED)#
find_package(sensor_msgs REQUIRED)#
find_package(geometry_msgs REQUIRED)#
find_package(tf2 REQUIRED)#
find_package(tf2_ros REQUIRED)#
find_package(tf2_msgs REQUIRED)#
find_package(tf2_sensor_msgs REQUIRED)#
find_package(tf2_geometry_msgs REQUIRED)#

## pcl install
#cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
cmake_policy(SET CMP0074 NEW)
find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(plane_filter src/ground_removal.cpp)
target_link_libraries(plane_filter ${PCL_LIBRARIES})
target_link_libraries(plane_filter
  ${Boost_SYSTEM_LIBRARY}  # pcl_conversions requires this
)

ament_target_dependencies(
  plane_filter
  geometry_msgs #
  pcl_conversions#
  rcl_interfaces#
  pcl_ros#
  rclcpp #
  rcutils
  sensor_msgs#
  std_msgs
  tf2_geometry_msgs#
  tf2_sensor_msgs#
  tf2_msgs#
  tf2_ros#
)

install(TARGETS
  plane_filter
  DESTINATION lib/${PROJECT_NAME}/
)


ament_export_dependencies(libpcl-all-dev pcl_ros pcl_conversions sensor_msgs)
ament_package()


