cmake_minimum_required(VERSION 3.18)
project(speed_annotator LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hdmap_core CONFIG REQUIRED)

add_executable(speed_annotator_node src/speed_annotator_node.cpp)
target_compile_features(speed_annotator_node PRIVATE cxx_std_17)
target_link_libraries(speed_annotator_node hdmap::core)
ament_target_dependencies(speed_annotator_node rclcpp interfaces std_msgs)

install(TARGETS speed_annotator_node RUNTIME DESTINATION lib/${PROJECT_NAME})
ament_package()
