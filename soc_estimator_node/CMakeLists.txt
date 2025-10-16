cmake_minimum_required(VERSION 3.8)
project(soc_estimator_node)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ROS2 패키지 설정
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_msg REQUIRED)
find_package(nav_msgs REQUIRED) 
find_package(geometry_msgs REQUIRED)

# Simulink & ROS2 코드 소스 등록
set(SOURCES
  src/BMS.cpp
  src/BMS_data.cpp
  src/soc_estimator_node.cpp
  src/main.cpp
)

# 실행 파일 생성
add_executable(soc_estimator_node_exec ${SOURCES})

# 🔧 include 경로 명확히 지정
target_include_directories(soc_estimator_node_exec PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/soc_estimator_node
)


# 종속성 연결
ament_target_dependencies(soc_estimator_node_exec
  rclcpp
  std_msgs
  ros2_msg
  nav_msgs 
  geometry_msgs
)

# 설치 설정
install(TARGETS
  soc_estimator_node_exec
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
