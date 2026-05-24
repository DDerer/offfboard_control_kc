cmake_minimum_required(VERSION 3.8)
project(offboard_control_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(offboard_node src/offboard_node.cpp)
ament_target_dependencies(offboard_node rclcpp px4_msgs)
target_compile_features(offboard_node PUBLIC cxx_std_17)

add_executable(simple_takeoff_node src/simple_takeoff_node.cpp)
ament_target_dependencies(simple_takeoff_node rclcpp px4_msgs)
target_compile_features(simple_takeoff_node PUBLIC cxx_std_17)

install(TARGETS
  offboard_node
  simple_takeoff_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
