cmake_minimum_required(VERSION 3.5)
project(diux)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(ros2_serializer
  src/ros2_serializer.cpp
  src/ros2_serial_transport.cpp
)
ament_target_dependencies(ros2_serializer
  rclcpp
  std_msgs
)

add_executable(ros2_serial_write
  src/ros2_serial_write.cpp
  src/ros2_serial_transport.cpp
)

install(TARGETS
  ros2_serializer
  ros2_serial_write
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
