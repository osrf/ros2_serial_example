#include <iostream>
#include <memory>

#include <termios.h>

#include "ros2_serial_transport.hpp"

int main(int argc, char *argv[])
{
  std::unique_ptr<Transport_node> transport_node = std::make_unique<UART_node>("/dev/ttyACM0", B115200, 0);

  transport_node->init();

  return 0;
}
