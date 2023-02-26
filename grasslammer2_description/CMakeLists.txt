cmake_minimum_required(VERSION 3.8)
project(grasslammer2_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


include_directories(include)

add_executable(tf_builder src/tf_builder.cpp)
ament_target_dependencies(
    tf_builder
    nav_msgs
    geometry_msgs
    rclcpp
    tf2
    tf2_ros
    
)
install(TARGETS
    tf_builder
    DESTINATION lib/${PROJECT_NAME})


install(
  DIRECTORY launch meshes urdf worlds models config src
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
