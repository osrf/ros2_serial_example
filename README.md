# ROS 2 serial bridge

## Purpose

This repository aims to provide a starting point (and possibly canonical version) of a serial <-> ROS 2 bridge.  The aim of the bridge is to take data to and from a serial port, and present that data on a ROS 2 network.

As of this writing (2019-03-22), there are several other projects in this space:

1.  px4_ros_com - https://github.com/PX4/px4_ros_com
1.  px4_to_ros - https://github.com/eProsima/px4_to_ros
1.  microROS - https://github.com/microROS
1.  microXRCE - https://github.com/eProsima/Micro-XRCE-DDS
1.  ros2arduino - https://github.com/ROBOTIS-GIT/ros2arduino

This project was originally forked out of px4_ros_com, but has since been heavily modified.

## Theory of operation

The serial-to-ROS 2 bridge contained in this repository (as the `ros2_to_serial_bridge` binary) defines serial framing protocols (the details of which are explained further in [Serial Framing Protocol](#Serial-Framing-Protocol).  Data that is properly framed coming *from* the serial port is unpacked and sent onto the ROS 2 network.  Conversely, data coming from the ROS 2 network is properly framed and sent *to* the serial port.  In all cases, the goal is to keep the amount of code on the other end of the serial port low so that this can be used to talk to embedded devices.

To keep the overhead of the serial framing low, there is a fixed-size "topic_ID" sent and received as part of the serial framing protocol.  This "topic_ID" represents a (topic name,topic type) tuple on the ROS 2 network.  By default, the "topic_ID" size is one byte, with 0 and 1 reserved, so that the maximum number of (topic_name,topic_type) tuples on any one bridge is 253.  There is a typedef in the code to increase this, but note that changing this will break serial wire compatibility.  The mapping of topic_ID -> (topic_name,topic_type) is discussed in the next section.

## Topic ID to name and type

As stated above, the bridge needs to know the mapping of topic IDs to the (topic_name,topic_type) tuple.  It can get this information either via a static YAML configuration file, or dynamically by querying the other side of the serial port.

### Static YAML configuration

If static YAML configuration is configured, then the topic ID -> (topic_name,topic_type) mapping comes from the file passed to the `ros2_serial_bridge` on the command line.  Static YAML configuration can be requested by setting the `dynamic_serial_mapping_ms` key in the YAML file to -1.  If static YAML configuration is configured, then no dynamic mapping is applied.

An example YAML file is presented in [ros2_serial_example/config/default_ros2_to_serial_bridge_params.yaml](ros2_serial_example/config/default_ros2_to_serial_bridge_params.yaml).  We'll gloss over the first part of the file here to concentrate on the topic mappings; see [YAML config](#YAML-config) for more information about the rest of the configuration file.

There's a top-level `topics` key, and under that are topic mappings of the form:

```
<topic_name>:
    serial_mapping: <serial_byte_mapping>
    type: <ROS2_type_mapping>
    direction: [SerialToROS2|ROS2ToSerial]
```

Data coming from the serial port with topic_ID `<serial_byte_mapping>` with direction `SerialToROS2` will be published on the ROS 2 network on topic `<topic_name>` with type `<ROS2_type_mapping>`.  Data coming from the ROS 2 network on topic `topic_name` with direction `ROS2ToSerial` with type `<ROS2_type_mapping>` will be framed onto the serial port with mapping `<serial_byte_mapping>`.  For maximum disambiguation, a topic_ID is exclusively either `SerialToROS2` or `ROS2ToSerial`.  This isn't a fundamental requirement of the protocol, so it could be lifted if necessary.

### Dynamic topic mapping

If dynamic topic mapping is configured, then the topic ID -> (topic_name,topic_type) mapping is queried over the serial port when `ros2_to_serial_bridge` starts.  Dynamic topic mapping can be requested by setting the `dynamic_serial_mapping_ms` key in the YAML configuration file to 0 or greater.  If 0, then `ros2_to_serial_bridge` will try "forever" to get the mapping via the serial port.  If greater than 0, then `ros2_to_serial_bridge` will try for that many milliseconds to get the mapping.  If it doesn't get it in that time, it quits the program.  If dynamic topic mapping is configured, then any static mapping in the YAML configuration file is completely ignored.

## Supported types

The message types that the bridge supports must be known at compile time. The CMake variable `ROS2_SERIAL_PKGS` is used to add entire packages to the list of supported messages; all messages in the particular package will be built into the bridge. For example, to add in all messages in `std_msgs`, `std_msgs` would be added to the `ROS2_SERIAL_PKGS` variable using this arguments: `--cmake-args -DROS2_SERIAL_PKGS="sensor_msgs"`. If you want to add more packages you can use `;` to separate them: `--cmake-args -DROS2_SERIAL_PKGS="sensor_msgs;px4_msgs"`. Each package type added to the bridge consumes more compile time and more on-disk space. The memory usage depends on which message types are setup during the topic mapping phase above. Note that if the topic mapping specifies a type that has not been compiled into `ros2_to_serial_bridge`, that topic will just be ignored.

## Using the code in this repository

### Build

1.  Install ROS 2 (https://index.ros.org/doc/ros2/Installation/).
1.  Source the ROS 2 installation (either `/opt/ros/<rosdistro>/setup.bash` if installing from binaries, or `ros2_ws/install/setup.bash` if building from source):
    1.  `source /opt/ros/crystal/setup.bash`
1.  Make a new workspace and clone this repository into it:
    1.  `mkdir -p ros2_serial_example_ws/src`
    1.  `cd ros2_serial_example/src`
    1.  `git clone https://github.com/osrf/ros2_serial_example.git`
    1.  `cd ..`
1.  Build the local workspace:
    1.  `colcon build --cmake-args -DROS2_SERIAL_PKGS="sensor_msgs;px4_msgs"`
1.  Source the local workspace:
    1.  `source install/local_setup.bash`

### Run

In terminal one:

`socat -d -d pty,raw,echo=0 pty,raw,echo=0`

socat will print two ptys out like /dev/pts/25 and /dev/pts/26; put the first one in the config file src/ros2_serial_example/ros2_serial_example/config/default_ros2_to_serial_bridge_params.yaml, and we'll use the second one below.

In terminal two, run the serial <-> RTPS bridge:

`./install/ros2_serial_example/lib/ros2_serial_example/ros2_to_serial_bridge __params:=src/ros2_serial_example/ros2_serial_example/config/default_ros2_to_serial_bridge_params.yaml`

In terminal three, send some data into the serial port that will show up in the ROS 2 network:

`./install/ros2_serial_example/lib/ros2_serial_example/dummy_serial -d /dev/pts/26`

Also in terminal three, send some data from the ROS 2 network that will end up on the serial port:

`ros2 topic pub -1 /another std_msgs/String "{data: 'hello'}"`

## Serial Framing Protocol

The current `ros2_to_serial_bridge` features two selectable serial protocols for transferring data over the serial link.  Both are intended to be simple and low overhead for the other end of the serial port to encode and decode (potentially a microcontroller).  The two supported protocols are:

1.  px4 - This protocol exists for 100% compatibility with the https://github.com/PX4/px4_ros_com project.  On the wire, the protocol looks like this (the vertical bars are octet separators and not actually part of the protocol):

```
>>>|topic_ID|seq|len_high|len_low|CRC_high|CRC_low|payload_start...payload_end|
```

The benefit to this protocol is that it is fairly simple, and provides some payload verification (via the CRC).  However, it suffers from the ability to disambiguate arbitrary data in the payload from another message (think about trying to send `>>>` in the message body).

2.  cobs - This protocol exists to fix the problems with the px4 protocol above.  On the wire, it looks like:

```
COBS(|topic_ID|len_high|len_low|CRC_high|CRC_low|payload_start...payload_end|)0x0
```

The COBS "stuffing" maps the 0-255 range of the octets into 1-255, leaving 0 available as an end-of-frame marker; see https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing .  With this protocol, it is easy to jump into the "middle" of a stream, and only lose one message; it is also possible to tunnel COBS over COBS.  Unless compatibility with PX4 is required, this protocol should be preferred.

## YAML Config

The YAML configuration file for the `ros2_to_serial_bridge` has a number of parameters that control how the bridge works:

* backend_comms - The type of comms layer to use for the "backend" communication (that is, the communication that is not ROS 2, which is considered frontend).  This is either 'uart' to use serial, or 'udp' to use a UDP socket.

* device - The /dev serial device to use to connect to the serial port.  This will be something like `/dev/ttyACM0` for USB-to-serial ports, or `/dev/pts/6` for "emulated" serial ports.  This is only used when backend_comms is 'uart'.

* baudrate - The baudrate to configure on the above device.  To skip baudrate configuration, set this to 0.  This is only used when backend_comms is 'uart'.

* udp_recv_port - The UDP port to use for receiving data.  This number must be between 1 and 65535 (inclusive).  This is only used when backend_comms is 'udp'.

* udp_send_port - The UDP prot to use for sending data.  This number must be between 1 and 65535 (inclusive).  This is only used when backend_comms is 'udp'.

* read_poll_ms - How many milliseconds to wait for new data to come in from the serial port.  Larger numbers can use slightly less CPU time, but affect the responsiveness of the program.  A value of 100 milliseconds is a good compromise between CPU time and responsiveness.

* ring_buffer_size - The number of bytes to use for the internal ring buffer for receiving data from the serial port.  Larger numbers allow larger messages to be taken from the serial port at the expense of memory.

* write_sleep_ms: How many milliseconds to sleep in between servicing ROS 2 callbacks.  Larger numbers will result in less CPU usage but also some latency in delivering data from ROS 2 to the serial port.  A value of 4 milliseconds is a good compromise between CPU time and latency.  It is not recommended to set this value larger than 100 milliseconds, as that can cause the application to feel sluggish.

* dynamic_serial_mapping_ms - How many milliseconds to wait on startup to get the dynamic ROS2-to-serial mapping from the serial port (see [Dynamic topic mapping](#Dynamic-topic-mapping) for more information).  If less than 0, dynamic mapping is disabled and the topics specified in the YAML configuration file are used.  If exactly 0, the bridge will wait forever for the serial side to respond, but note that no data transfer of topic data will start happening until this succeeds.  If greater than 0, wait that many milliseconds for a response from the serial port before failing to start.  If this number is greater than or equal to 0, the topics configured in the YAML file are completely ignored.

* backend_protocol - One of 'px4' or 'cobs'.  See [Serial Framing Protocol](#Serial-Framing-Protocol) for more information.

* topics - The list of topics to use if dynamic_serial_mapping_ms is less than 0.  See [Static YAML configuration](#Static-YAML-configuration) for more information.

## Code generation for the bridge

The way that the compile process generates code for the bridge is slightly complicated, so this section aims to shed some light on that process.

1.  The list of packages (`_packages`) and individual messages (`_msgs`) to generate type support for are collected by [CMakeLists.txt](ros2_serial_example/CMakeLists.txt) and passed to a [python script](ros2_serial_example/generate_ros2_topics.py) via `execute_process`.  At this point, the python script just returns the list of files that *would* be generated, but doesn't do any generation (this is so we can properly add dependencies to later CMake steps).

1.  The [python script](ros2_serial_example/generate_ros2_topics.py) that will eventually generate the type support via `add_custom_command` is created, with outputs corresponding to the list of files we got in the last step.

1.  The [CMakeLists.txt](ros2_serial_example/CMakeLists.txt) collects the message package dependencies for the bridge.  The message package dependencies are a combination of the packages specified in `_packages` plus the implied packages from the individual messages (basically the part before the first /).

1.  Now the bridge targets are configured, where the bridge sources consist of the generated files from above plus any bridge-specific sources.  The dependencies for the bridge are also set to the dependencies generated above.

1.  When the `ros2_to_serial_bridge` target is built, it will first run the dependency to generate the message type support.  This runs the [python script](ros2_serial_example/generate_ros2_topics.py), which will actually generate the sources at this point.

1.  The rest of the compilation will happen, with the generated sources getting included into the build and into the final `ros2_to_serial_bridge` binary.
