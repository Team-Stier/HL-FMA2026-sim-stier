cmake_minimum_required(VERSION 3.18)
project(control)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(control_core src/controller_core.cpp)
target_compile_features(control_core PUBLIC cxx_std_17)
target_include_directories(control_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

add_executable(mpc_control_node src/mpc_control_node.cpp)
target_compile_features(mpc_control_node PUBLIC cxx_std_17)
target_link_libraries(mpc_control_node control_core)
ament_target_dependencies(mpc_control_node interfaces nav_msgs rclcpp std_msgs)

install(TARGETS control_core mpc_control_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    enable_testing()
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_executable(control_core_test test/control_core_test.cpp)
    target_compile_features(control_core_test PUBLIC cxx_std_17)
    target_compile_options(control_core_test PRIVATE -UNDEBUG)
    target_link_libraries(control_core_test control_core)
    add_test(NAME control_core_test COMMAND control_core_test)
    add_test(NAME control_node_test COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/test/control_node_test.py $<TARGET_FILE:mpc_control_node>)
    set_tests_properties(control_node_test PROPERTIES TIMEOUT 40)
endif()

ament_export_include_directories(include)
ament_export_libraries(control_core)
ament_package()
