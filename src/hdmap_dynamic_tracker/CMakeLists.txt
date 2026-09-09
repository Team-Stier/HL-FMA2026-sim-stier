cmake_minimum_required(VERSION 3.18)
project(hdmap_dynamic_tracker LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(hdmap_core CONFIG REQUIRED)

add_library(tracker_core
    src/tracker_core.cpp
    src/ekf_predictor.cpp)
target_compile_features(tracker_core PUBLIC cxx_std_17)
target_include_directories(tracker_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_options(tracker_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(hdmap_dynamic_tracker_node src/hdmap_dynamic_tracker_node.cpp)
target_compile_features(hdmap_dynamic_tracker_node PRIVATE cxx_std_17)
target_compile_options(hdmap_dynamic_tracker_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(hdmap_dynamic_tracker_node tracker_core hdmap::core)
ament_target_dependencies(hdmap_dynamic_tracker_node rclcpp interfaces)

install(TARGETS tracker_core hdmap_dynamic_tracker_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

include(CTest)
if(BUILD_TESTING)
    add_executable(tracker_core_test test/tracker_core_test.cpp)
    target_compile_features(tracker_core_test PRIVATE cxx_std_17)
    target_compile_options(tracker_core_test PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
    target_link_libraries(tracker_core_test PRIVATE tracker_core)
    add_test(NAME tracker_core_test COMMAND tracker_core_test)
endif()

ament_export_include_directories(include)
ament_export_libraries(tracker_core)
ament_package()
