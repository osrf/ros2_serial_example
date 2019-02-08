# ROS 2 serial example

To build:

$ git clone https://github.com/eProsima/Fast-RTPS
$ sudo apt-get install default-jre gradle
$ mkdir -p Fast-RTPS/build
$ pushd Fast-RTPS/build
$ cmake -DTHIRDPARTY=ON -DBUILD_JAVA=ON -DCMAKE_INSTALL_PREFIX=../../install ..
$ make -j10 install
$ popd
$ make -j10

To run:
$ LD_LIBRARY_PATH=install/lib ./ros2_serializer
