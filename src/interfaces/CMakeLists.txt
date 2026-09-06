cmake_minimum_required(VERSION 3.18)
project(interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/EgoPose.msg"
    "msg/EgoStatus.msg"
    "msg/Objects.msg"
    "msg/TrafficLight.msg"
    "msg/DynamicStatus.msg"
    "msg/ControlCommand.msg"
    "msg/SearchTree.msg"
    "msg/CellGeometry.msg"
    "msg/CellColors.msg"
    DEPENDENCIES std_msgs geometry_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
