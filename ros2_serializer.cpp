#include <csignal>
#include <iostream>
#include <memory>

#include <termios.h>

#include "ros2_serial_transport.hpp"

volatile sig_atomic_t running = 1;

void signal_handler(int signum)
{
  (void)signum;
  running = 0;
}

int main(int argc, char *argv[])
{
  std::unique_ptr<Transport_node> transport_node = std::make_unique<UART_node>("/dev/ttyACM0", B115200, 0);

  signal(SIGINT, signal_handler);

  if (transport_node->init() < 0) {
    return 1;
  }

  while (running) {
  }

  transport_node->close();

  return 0;
}
