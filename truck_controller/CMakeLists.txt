cmake_minimum_required(VERSION 3.5)
project(truck_controller)


find_package(ament_cmake REQUIRED)
find_package(rclcpp       REQUIRED)
find_package(std_msgs     REQUIRED)
find_package(ros2_msg     REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


# 헤더 검색
include_directories(include)

include_directories(
  ${tf2_geometry_msgs_INCLUDE_DIRS}
)

# 실행 파일
add_executable(truck_controller_node
  src/main.cpp
  src/truck_controller.cpp
)

# ROS2 의존성 링크 (plain 시그니처)
ament_target_dependencies(
  truck_controller_node
  rclcpp
  std_msgs
  ros2_msg
  rclcpp std_msgs ros2_msg
  nav_msgs geometry_msgs
  sensor_msgs  
  tf2_geometry_msgs

)


target_link_libraries(
  truck_controller_node
  Geographic

)



# 설치
install(TARGETS truck_controller_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

