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
$ socat -d -d pty,raw,echo=0 pty,raw,echo=0
(socat will print two ptys out like /dev/pts/25 and /dev/pts/26)

Run the serial -> RTPS bridge:
$ LD_LIBRARY_PATH=install/lib ./ros2_serializer -d /dev/pts/25

To send some example data into the serial port:
$ LD_LIBRARY_PATH=install/lib ./ros2_serial_write -d /dev/pts/26
