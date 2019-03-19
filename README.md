# ROS 2 serial example

## Build

1.  Install ROS 2 (https://index.ros.org/doc/ros2/Installation/).
1.  Source the ROS 2 installation (either `/opt/ros/<rosdistro>/setup.bash` if installing from binaries, or `ros2_ws/install/setup.bash` if building from source):
    1.  `source /opt/ros/crystal/setup.bash`
1.  Make a new workspace and clone this repository into it:
    1.  `mkdir -p ros2_serial_example_ws/src`
    1.  `cd ros2_serial_example/src`
    1.  `git clone https://github.com/osrf/ros2_serial_example.git`
    1.  `cd ..`
1.  Build the local workspace:
    1.  `colcon build`
1.  Source the local workspace:
    1.  `source install/local_setup.bash`

## Run

In terminal one:

`socat -d -d pty,raw,echo=0 pty,raw,echo=0`

socat will print two ptys out like /dev/pts/25 and /dev/pts/26; put the first one in the config file src/ros2_serial_example/config/default_ros2_to_serial_bridge_params.yaml, and we'll use the second one below.

In terminal two, run the serial <-> RTPS bridge:

`./install/ros2_serial_example/lib/ros2_serial_example/ros2_to_serial_bridge __params:=src/ros2_serial_example/ros2_serial_example/config/default_ros2_to_serial_bridge_params.yaml`

In terminal three, send some data into the serial port that will show up in the ROS 2 network:

`./install/ros2_serial_example/lib/ros2_serial_example/dummy_serial -d /dev/pts/26`

Also in terminal three, send some data from the ROS 2 network that will end up on the serial port:

`ros2 topic pub -1 /another std_msgs/String "{data: 'hello'}"`
