CXX=g++
CXXFLAGS=-g -Wall -Wextra

all: ros2_serializer

ros2_serializer: ros2_serializer.o
	$(CXX) $(CXXFLAGS) -o $@ $<

clean:
	rm -f ros2_serializer *.o *~
