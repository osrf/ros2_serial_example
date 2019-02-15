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

include_directories(include)

add_executable(ros2_serializer
  src/ros2_serializer.cpp
  src/ros2_serial_transport.cpp
)
ament_target_dependencies(ros2_serializer
  fastcdr
  rclcpp
  std_msgs
)
target_link_libraries(ros2_serializer
  fastcdr
)

add_executable(ros2_serial_write
  src/ros2_serial_write.cpp
  src/ros2_serial_transport.cpp
)
ament_target_dependencies(ros2_serial_write
  fastcdr
)
target_link_libraries(ros2_serial_write
  fastcdr
)

install(TARGETS
  ros2_serializer
  ros2_serial_write
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
