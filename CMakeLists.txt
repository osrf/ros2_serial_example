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
find_package(Threads REQUIRED)

include_directories(include "${CMAKE_CURRENT_BINARY_DIR}")

# This is the set of packages to build the ROS2<->serial bridge for.  All
# messages in packages referenced by _packages will have support compiled in,
# and the individual messages mentioned in _msgs will also be compiled in.
#
# A few important notes:
# 1.  Be *very* wary of adding whole packages to this list.  For instance, with
#     just 'std_msgs', the build takes something like 1.8GB of RAM to compile
#     and the resulting binary is 38MB.  Each additional package causes both of
#     those numbers to go up dramatically.
# 2.  If you add a new package or idl file here, you *must* remember to
#     "find_package" it above, and add it to package.xml since this package now
#     depends on it.
set(_packages
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
  COMMAND ${PYTHON_EXECUTABLE} ${_generator} ${_tmpl_dir} ${_output_dir} --print-outputs ${_flags}
  OUTPUT_VARIABLE _generated_sources
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

add_custom_command(
  OUTPUT ${_generated_sources}
  COMMAND ${PYTHON_EXECUTABLE} ${_generator} ${_tmpl_dir} ${_output_dir} ${_flags}
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

add_executable(ros2_to_serial_bridge
  src/ros2_to_serial_bridge.cpp
  src/transporter.cpp
  src/uart_transporter.cpp
  ${_generated_sources}
)
ament_target_dependencies(ros2_to_serial_bridge
  fastcdr
  rclcpp
  ${_deps}
)
target_link_libraries(ros2_to_serial_bridge
  fastcdr
)

add_executable(dummy_serial
  src/dummy_serial.cpp
  src/transporter.cpp
  src/uart_transporter.cpp
)
ament_target_dependencies(dummy_serial
  fastcdr
)
target_link_libraries(dummy_serial
  fastcdr
  ${CMAKE_THREAD_LIBS_INIT}
)

install(TARGETS
  dummy_serial
  ros2_to_serial_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
