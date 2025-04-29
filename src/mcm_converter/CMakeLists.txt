cmake_minimum_required(VERSION 3.5)
project(mcm_converter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
  asn1/generated
)

add_executable(converter_node src/converter_node.cpp
  asn1/generated/MCM.c
  asn1/generated/ManeuverCoordinationMessage.c
  asn1/generated/BasicContainerMCM.c
  # alle weiteren .c-Dateien aus dem generated Ordner
)

target_link_libraries(converter_node m)

ament_target_dependencies(converter_node rclcpp std_msgs)

install(TARGETS
  converter_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

