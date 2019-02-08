CXX=g++
CXXFLAGS=-g -Wall -Wextra -std=c++14 -I install/include
LIBS=-Linstall/lib -lfastcdr -lfastrtps
SERIALIZER_OBJS=ros2_serializer.o ros2_serial_transport.o RtpsTopics.o battery_status_PubSubTypes.o battery_status_Publisher.o battery_status_.o

all: ros2_serializer ros2_serial_write

ros2_serializer: $(SERIALIZER_OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(SERIALIZER_OBJS) $(LIBS)

ros2_serial_write: ros2_serial_write.o ros2_serial_transport.o
	$(CXX) $(CXXFLAGS) -o $@ ros2_serial_write.o ros2_serial_transport.o

clean:
	rm -f ros2_serializer *.o *~

allclean: clean
	rm -rf install
