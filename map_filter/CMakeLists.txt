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
#openMP
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
  set(OpenMP_LIBS gomp)
endif()

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
  OpenMP
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

add_executable(cropbox_cloud src/cropbox_cloud.cpp)
target_link_libraries(cropbox_cloud ${PCL_LIBRARIES})
target_link_libraries(cropbox_cloud
  ${Boost_SYSTEM_LIBRARY}  # pcl_conversions requires this
)
ament_target_dependencies(
  cropbox_cloud
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
  cropbox_cloud
  DESTINATION lib/${PROJECT_NAME}/
)

add_executable(cloud_to_scan src/cloud_to_scan.cpp)
target_link_libraries(cloud_to_scan ${PCL_LIBRARIES})
target_link_libraries(cloud_to_scan
  ${Boost_SYSTEM_LIBRARY}  # pcl_conversions requires this
)
ament_target_dependencies(
  cloud_to_scan
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
  cloud_to_scan
  DESTINATION lib/${PROJECT_NAME}/
)

add_executable(map_filter_sectors src/map_filter_sectors.cpp)
target_link_libraries(map_filter_sectors ${PCL_LIBRARIES})
target_link_libraries(map_filter_sectors
  ${Boost_SYSTEM_LIBRARY}  # pcl_conversions requires this
)
ament_target_dependencies(
  map_filter_sectors
  OpenMP
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
  map_filter_sectors
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

target_compile_options(map_filter PRIVATE ${OpenMP_FLAGS})
target_link_libraries(map_filter ${OpenMP_LIBS})
target_compile_options(map_filter_sectors PRIVATE ${OpenMP_FLAGS})
target_link_libraries(map_filter_sectors ${OpenMP_LIBS})

ament_package()