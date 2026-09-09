cmake_minimum_required(VERSION 3.18)
project(path_planner LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hdmap_core CONFIG REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_routing REQUIRED)
find_package(lanelet2_traffic_rules REQUIRED)

add_executable(path_planner_node src/path_planner_node.cpp)
target_compile_features(path_planner_node PRIVATE cxx_std_17)
target_compile_options(path_planner_node PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(path_planner_node PRIVATE
  ${lanelet2_core_INCLUDE_DIRS}
  ${lanelet2_routing_INCLUDE_DIRS}
  ${lanelet2_traffic_rules_INCLUDE_DIRS})
target_link_libraries(path_planner_node hdmap::core
  ${lanelet2_core_LIBRARIES}
  ${lanelet2_routing_LIBRARIES}
  ${lanelet2_traffic_rules_LIBRARIES})
ament_target_dependencies(path_planner_node rclcpp interfaces std_msgs nav_msgs geometry_msgs)

install(TARGETS path_planner_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch checkpoint DESTINATION share/${PROJECT_NAME})
ament_package()
