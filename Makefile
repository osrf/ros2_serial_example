CXX=g++
CXXFLAGS=-g -Wall -Wextra -std=c++14 -I install/include -I /home/clalancette/ros2_ws/install/rclcpp/include -I /home/clalancette/ros2_ws/install/rcl/include -I /home/clalancette/ros2_ws/install/rcutils/include -I /home/clalancette/ros2_ws/install/rmw/include -I /home/clalancette/ros2_ws/install/rosidl_generator_c/include -I /home/clalancette/ros2_ws/install/rosidl_typesupport_interface/include -I /home/clalancette/ros2_ws/install/rosidl_generator_cpp/include -I /home/clalancette/ros2_ws/install/rcl_interfaces/include -I /home/clalancette/ros2_ws/install/builtin_interfaces/include -I /home/clalancette/ros2_ws/install/std_msgs/include
LIBS=-L install/lib -L /home/clalancette/ros2_ws/install/rclcpp/lib -L /home/clalancette/ros2_ws/install/rcutils/lib -L /home/clalancette/ros2_ws/install/rcl/lib -L /home/clalancette/ros2_ws/install/std_msgs/lib -lfastcdr -lfastrtps -lrclcpp -lrcutils -lrcl -lstd_msgs__rosidl_typesupport_cpp
SERIALIZER_OBJS=ros2_serializer.o ros2_serial_transport.o
RTPSRECV_OBJS=ros2_rtps_recv.o ros2_serial_transport.o battery_status_Subscriber.o battery_status_PubSubTypes.o battery_status_.o RtpsTopics.o battery_status_Publisher.o string_.o string_PubSubTypes.o string_Subscriber.o string_Publisher.o

all: ros2_serializer ros2_serial_write ros2_rtps_recv

ros2_serializer: $(SERIALIZER_OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(SERIALIZER_OBJS) $(LIBS)

ros2_serial_write: ros2_serial_write.o ros2_serial_transport.o
	$(CXX) $(CXXFLAGS) -o $@ ros2_serial_write.o ros2_serial_transport.o

ros2_rtps_recv: $(RTPSRECV_OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(RTPSRECV_OBJS) $(LIBS)

clean:
	rm -f ros2_serializer *.o *~ ros2_rtps_recv ros2_serial_write

allclean: clean
	rm -rf install
