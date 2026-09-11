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
if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME speed_annotator_test COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/speed_annotator_test.py
    $<TARGET_FILE:speed_annotator_node> ${CMAKE_CURRENT_SOURCE_DIR}/../../map/hdmap.bin
    ${hdmap_core_DIR}/../../../lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages)
  set_tests_properties(speed_annotator_test PROPERTIES TIMEOUT 40)
endif()
ament_package()
