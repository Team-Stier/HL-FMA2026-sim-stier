cmake_minimum_required(VERSION 3.8)
project(sim_bridge)
find_package(ament_cmake REQUIRED)
install(PROGRAMS sim_bridge_node.py DESTINATION lib/${PROJECT_NAME} RENAME sim_bridge_node)
install(FILES ../../config/runtime.yaml ../../config/vehicle.yaml ../../config/object_offsets.yaml
    DESTINATION share/${PROJECT_NAME}/config)
ament_package()
