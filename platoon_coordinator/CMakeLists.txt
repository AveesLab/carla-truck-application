cmake_minimum_required(VERSION 3.5)
project(platoon_coordinator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_msg REQUIRED)

include_directories(include)

add_executable(platoon_coordinator src/main.cpp src/platoon_coordinator.cpp)
ament_target_dependencies(platoon_coordinator
  rclcpp
  std_msgs
  ros2_msg
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/ros2_msg/msg>
)


install(TARGETS platoon_coordinator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
