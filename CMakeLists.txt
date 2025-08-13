cmake_minimum_required(VERSION 3.16)
project(contact_sensor_hardware_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ros2_control REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/contact_sensor_hardware_interface.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ros2_control
)
target_link_libraries(${PROJECT_NAME} pthread)

pluginlib_export_plugin_description_file(hardware_interface contact_sensor_hardware_interface.xml)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)
install(FILES contact_sensor_hardware_interface.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY urdf/   DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
