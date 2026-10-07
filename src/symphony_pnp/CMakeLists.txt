cmake_minimum_required(VERSION 3.8)
project(symphony_pnp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(pnp src/pnp.cpp)
ament_target_dependencies(pnp rclcpp moveit_ros_planning_interface tf2_geometry_msgs)

find_package(ignition-gazebo6 QUIET)
if(ignition-gazebo6_FOUND)
  add_library(pnp_hold SHARED src/pnp_hold.cpp)
  target_link_libraries(pnp_hold ignition-gazebo6::ignition-gazebo6)
  install(TARGETS pnp_hold DESTINATION lib)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS pnp DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
  launch models 
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
