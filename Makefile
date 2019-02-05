CXX=g++
CXXFLAGS=-g -Wall -Wextra -std=c++14

all: ros2_serializer

ros2_serializer: ros2_serializer.o ros2_serial_transport.o
	$(CXX) $(CXXFLAGS) -o $@ ros2_serializer.o ros2_serial_transport.o

clean:
	rm -f ros2_serializer *.o *~
