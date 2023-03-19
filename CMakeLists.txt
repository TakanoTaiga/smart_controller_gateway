cmake_minimum_required(VERSION 3.5)
project(smart_controller_gateway)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_smart_controller_gateway SHARED
  DIRECTORY src
)

target_include_directories(target_smart_controller_gateway
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_smart_controller_gateway
  PLUGIN "smart_controller_gateway::SmartControllerGatewayNode"
  EXECUTABLE smart_controller_gateway_node
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)