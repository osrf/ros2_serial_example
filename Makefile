CXX=g++
CXXFLAGS=-g -Wall -Wextra -std=c++14 -I install/include
LIBS=-Linstall/lib -lfastcdr -lfastrtps
SERIALIZER_OBJS=ros2_serializer.o ros2_serial_transport.o RtpsTopics.o battery_status_PubSubTypes.o battery_status_Publisher.o battery_status_.o battery_status_Subscriber.o string_.o string_PubSubTypes.o string_Subscriber.o string_Publisher.o
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
