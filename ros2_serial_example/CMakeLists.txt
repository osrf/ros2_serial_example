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
find_package(ros2_serial_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

include_directories(include "${CMAKE_CURRENT_BINARY_DIR}")

if (ROS2_SERIAL_PKGS)
  foreach(pkgs ${ROS2_SERIAL_PKGS})
    find_package(${pkgs} REQUIRED)
  endforeach(pkgs)
endif()

# This is the set of packages to build the ROS2<->serial bridge for.  All
# messages in packages referenced by _packages will have support compiled in,
# and the individual messages mentioned in _msgs will also be compiled in.
#
# If a new package or message is added here, the corresponding package must
# be "find_package"d above and added to package.xml.
set(_packages
  ${ROS2_SERIAL_PKGS}
  std_msgs
)
set(_msgs
)

# Given the packages and messages above, create the target for code generation.
set(_flags)
if (NOT "${_packages}" STREQUAL "")
  set(_flags "${_flags}" "--packages" "${_packages}")
endif()
if (NOT "${_msgs}" STREQUAL "")
  set(_flags "${_flags}" "--ros2-msgs" "${_msgs}")
endif()

set(_generator "${CMAKE_CURRENT_SOURCE_DIR}/generate_ros2_topics.py")
set(_tmpl_dir "${CMAKE_CURRENT_SOURCE_DIR}/templates")
set(_output_dir "${CMAKE_CURRENT_BINARY_DIR}")
# TODO(clalancette): CMake will *not* reexecute this if the generator file changes.
execute_process(
  COMMAND ${Python3_EXECUTABLE} ${_generator} ${_tmpl_dir} ${_output_dir} --print-outputs ${_flags}
  OUTPUT_VARIABLE _generated_sources
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

add_custom_command(
  OUTPUT ${_generated_sources}
  COMMAND ${Python3_EXECUTABLE} ${_generator} ${_tmpl_dir} ${_output_dir} ${_flags}
  DEPENDS ${_generator} ${_tmpl_dir}/ros2_topics.hpp.em ${_tmpl_dir}/pub_sub_type.hpp.em ${_tmpl_dir}/pub_sub_type.cpp.em
  COMMENT "Generating topics"
)

# Now setup the ament_target_dependencies for below.
set(_deps ${_packages})
foreach(msg ${_msgs})
  string(REPLACE "/" ";" msglist ${msg})
  list(LENGTH msglist msglen)
  if (NOT ${msglen} EQUAL 2)
    message(FATAL_ERROR "Invalid message format ${msg}, should be <package>/<name>")
  endif()
  list(GET msglist 0 pkg)
  list(APPEND _deps ${pkg})
endforeach()
list(REMOVE_DUPLICATES _deps)

# Now setup the libraries for below.
set(_libs)
foreach(pkg ${_packages})
  list(APPEND _libs ${${pkg}_LIBRARIES__rosidl_typesupport_fastrtps_cpp})
endforeach()
list(REMOVE_DUPLICATES _libs)

add_library(ring_buffer
  src/ring_buffer.cpp
)

add_library(transporter
  src/transporter.cpp
)
target_link_libraries(transporter
  ring_buffer
)

add_library(bridge_gen
  ${_generated_sources}
)
ament_target_dependencies(bridge_gen
  rclcpp
  ${_deps}
)
target_link_libraries(bridge_gen
  fastcdr
  ${_libs}
)

add_library(ros2_to_serial_bridge SHARED
  src/ros2_to_serial_bridge.cpp
  src/uart_transporter.cpp
  src/udp_transporter.cpp
)
ament_target_dependencies(ros2_to_serial_bridge
  "rclcpp"
  "rclcpp_components"
  "ros2_serial_msgs")
target_link_libraries(ros2_to_serial_bridge
  bridge_gen
  fastcdr
  ring_buffer
  transporter
)
rclcpp_components_register_node(
  ros2_to_serial_bridge
  PLUGIN "ros2_to_serial_bridge::ROS2ToSerialBridge"
  EXECUTABLE ros2_to_serial_bridge_node
)

add_executable(dummy_serial
  src/dummy_serial.cpp
  src/uart_transporter.cpp
)
ament_target_dependencies(dummy_serial
  ros2_serial_msgs
)
target_link_libraries(dummy_serial
  fastcdr
  ring_buffer
  transporter
  ${ros2_serial_msgs_LIBRARIES__rosidl_typesupport_fastrtps_cpp}
  ${CMAKE_THREAD_LIBS_INIT}
)

add_executable(dummy_udp
  src/dummy_udp.cpp
  src/udp_transporter.cpp
)
ament_target_dependencies(dummy_udp
  ros2_serial_msgs
)
target_link_libraries(dummy_udp
  fastcdr
  ring_buffer
  transporter
  ${ros2_serial_msgs_LIBRARIES__rosidl_typesupport_fastrtps_cpp}
  ${CMAKE_THREAD_LIBS_INIT}
)

install(TARGETS ring_buffer transporter bridge_gen
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  dummy_serial
  dummy_udp
  ros2_to_serial_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ros2_to_serial_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_ring_buffer test/test_ring_buffer.cpp)
  target_link_libraries(test_ring_buffer ring_buffer)

  ament_add_gtest(test_transporter test/test_transporter.cpp)
  target_link_libraries(test_transporter transporter)

  ament_add_gtest(test_ros2_topics test/test_ros2_topics.cpp)
  target_link_libraries(test_ros2_topics transporter bridge_gen)
endif()

ament_package()
