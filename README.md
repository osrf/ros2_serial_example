# ROS 2 serial example

## Build

1.  Install ROS 2 (https://index.ros.org/doc/ros2/Installation/).
1.  Source the ROS 2 installation (either /opt/ros/<rosdistro>/setup.bash if installing from binaries, or ros2_ws/install/setup.bash if building from source).
1.  Make a new workspace and clone this repository into it: mkdir -p ros2_serial_example_ws/src ; cd ros2_serial_example/src ; git clone https://github.com/osrf/ros2_serial_example.git ; cd ..
1.  Build the local workspace: colcon build
1.  Source the local workspace: . install/local_setup.bash

## Run

In terminal one:
$ socat -d -d pty,raw,echo=0 pty,raw,echo=0
(socat will print two ptys out like /dev/pts/25 and /dev/pts/26)

In terminal two, run the serial -> RTPS bridge:
$ ./install/ros2_serial_example/lib/ros2_serial_example/ros2_serializer -d /dev/pts/25

In terminal three, send some example data into the serial port:
$ ./install/ros2_serial_example/lib/ros2_serial_example/ros2_serial_write -d /dev/pts/26
