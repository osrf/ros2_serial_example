cmake_minimum_required(VERSION 3.5)
project(ros2_serial_example)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -ggdb3)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(fastcdr REQUIRED CONFIG)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include "${CMAKE_CURRENT_BINARY_DIR}")

set(_generator "${CMAKE_CURRENT_SOURCE_DIR}/generate_ros2_topics.py")
set(_ros2_topics "${CMAKE_CURRENT_BINARY_DIR}/ros2_topics.hpp")
set(_ros2_topics_tmpl "${CMAKE_CURRENT_SOURCE_DIR}/ros2_topics.hpp.em")
add_custom_command(
  OUTPUT ${_ros2_topics}
  #COMMAND ${PYTHON_EXECUTABLE} ${_generator} ${_ros2_topics_tmpl} ${_ros2_topics} --packages std_msgs
  COMMAND ${PYTHON_EXECUTABLE} ${_generator} ${_ros2_topics_tmpl} ${_ros2_topics} --idl-files /home/clalancette/ros2_ws/install/std_msgs/share/std_msgs/msg/String.idl
  DEPENDS ${_generator} ${_ros2_topics_tmpl}
  COMMENT "Generating ros2_topics.hpp"
)

add_executable(ros2_serializer
  src/ros2_serializer.cpp
  src/transporter.cpp
  src/uart_transporter.cpp
  ${_ros2_topics}
)
ament_target_dependencies(ros2_serializer
  fastcdr
  rclcpp
  std_msgs
)
target_link_libraries(ros2_serializer
  fastcdr
)

add_executable(serial_writer
  src/serial_writer.cpp
  src/transporter.cpp
  src/uart_transporter.cpp
)
ament_target_dependencies(serial_writer
  fastcdr
)
target_link_libraries(serial_writer
  fastcdr
)

install(TARGETS
  ros2_serializer
  serial_writer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
