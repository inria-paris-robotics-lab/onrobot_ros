cmake_minimum_required(VERSION 3.8)
project(onrobot_interface)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ur_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(class_loader REQUIRED)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)


set(SOURCES
  src/onrobot_gripper.cpp
  src/onrobot_interface.cpp
)

# Library for the hardware interface
add_library(${PROJECT_NAME} SHARED ${SOURCES})
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  controller_interface
  ament_index_cpp
  class_loader
  ur_msgs
  std_msgs
)
pluginlib_export_plugin_description_file(hardware_interface onrobot_interface.xml)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install shared lib
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES onrobot_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()

# include_directories(
#   include
#   ${CMAKE_CURRENT_SOURCE_DIR}/include
# )

# add_executable(test_node
#   src/test.cpp
#   ${DRIVER_SOURCES}
# )
# ament_target_dependencies(test_node
#   rclcpp
#   std_msgs
#   ur_msgs
# )
# install(TARGETS test_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# ament_package()
