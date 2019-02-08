#include <csignal>
#include <iostream>
#include <memory>

#include <termios.h>
#include <unistd.h>

#include "ros2_serial_transport.hpp"
#include "RtpsTopics.hpp"

constexpr int BUFFER_SIZE = 1024;

volatile sig_atomic_t running = 1;

void signal_handler(int signum)
{
  (void)signum;
  running = 0;
}

int main(int argc, char *argv[])
{
  (void)argc;
  (void)argv;

  std::unique_ptr<Transport_node> transport_node = std::make_unique<UART_node>("/dev/ttyACM0", B115200, 0);

  ::signal(SIGINT, signal_handler);

  if (transport_node->init() < 0) {
    return 1;
  }

  ::sleep(1);

  char data_buffer[BUFFER_SIZE] = {};
  int length = 0;
  uint8_t topic_ID = 255;

  RtpsTopics topics;

  topics.init();

  running = 1;

  while (running) {
    while ((length = transport_node->read(&topic_ID, data_buffer, BUFFER_SIZE)) > 0) {
      topics.publish(topic_ID, data_buffer, sizeof(data_buffer));
    }
    ::usleep(1);
  }

  transport_node->close();

  return 0;
}
