cmake_minimum_required(VERSION 3.8)
project(traffic_planner)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

# TrafficPlanner 알고리즘 라이브러리 (ROS 독립적)
add_library(traffic_planner_lib
  src/TrafficPlanner.cpp
)

# 기존 TrafficPlannerNode 라이브러리 (ROS2 의존적)
add_library(traffic_planner_node_lib
  src/TrafficPlannerNode.cpp
)

# 새로운 SynchronizedTrafficPlannerNode 라이브러리 (ROS2 의존적)
add_library(synchronized_traffic_planner_node_lib
  src/SynchronizedTrafficPlannerNode.cpp
)

# 기존 실행 파일
add_executable(traffic_path_planner
  src/main.cpp
)

# 새로운 동기화 실행 파일
add_executable(synchronized_traffic_path_planner
  src/main_synchronized.cpp
)

# 라이브러리 링크 - 기존
target_link_libraries(traffic_planner_node_lib traffic_planner_lib)
target_link_libraries(traffic_path_planner traffic_planner_node_lib)

# 라이브러리 링크 - 새로운 동기화 버전
target_link_libraries(synchronized_traffic_planner_node_lib traffic_planner_lib)
target_link_libraries(synchronized_traffic_path_planner synchronized_traffic_planner_node_lib)

# ROS2 의존성 - 기존
ament_target_dependencies(traffic_planner_node_lib
  rclcpp
  geometry_msgs
  nav_msgs
)

ament_target_dependencies(traffic_path_planner
  rclcpp
  geometry_msgs
  nav_msgs
)

# ROS2 의존성 - 새로운 동기화 버전
ament_target_dependencies(synchronized_traffic_planner_node_lib
  rclcpp
  geometry_msgs
  nav_msgs
)

ament_target_dependencies(synchronized_traffic_path_planner
  rclcpp
  geometry_msgs
  nav_msgs
)

# 설치
install(TARGETS 
  traffic_path_planner 
  synchronized_traffic_path_planner
  traffic_planner_lib 
  traffic_planner_node_lib
  synchronized_traffic_planner_node_lib
  DESTINATION lib/${PROJECT_NAME}
)

# 헤더 파일 설치 (다른 패키지에서 사용할 수 있도록)
install(DIRECTORY include/
  DESTINATION include/
)

# Launch 파일 설치
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

ament_package()