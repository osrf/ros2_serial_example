#include <cstdio>
#include <cstring>
#include <memory>

#include <termios.h>
#include <unistd.h>

#include "ros2_serial_example/ros2_serial_transport.hpp"

static void usage(const char *name)
{
    ::printf("Usage: %s [options]\n\n"
             "  -d <device> UART device. Default /dev/ttyACM0\n"
             "  -h          Print this help message\n",
             name);
}

int main(int argc, char *argv[])
{
    char device[64] = "/dev/ttyACM0";

    int ch;
    while ((ch = ::getopt(argc, argv, "d:h")) != EOF)
    {
      switch (ch)
      {
      case 'd':
          if (optarg != nullptr)
          {
              ::strcpy(device, optarg);
          }
          break;
      case 'h':
          usage(argv[0]);
          return 0;
      default:
          usage(argv[0]);
          return 1;
      }
    }

    if (optind < argc)
    {
      usage(argv[0]);
      return 1;
    }

    std::unique_ptr<Transport_node> transport_node = std::make_unique<UART_node>(device, B115200, 0);

    if (transport_node->init() < 0)
    {
      return 1;
    }

    // 9 bytes for header, plus 83 bytes for battery_status message
    uint8_t buffer[9 + 8 + 15*4 + 5*2 + 5*1] = {};

    buffer[9] = 0xaa;

    transport_node->write(6, buffer, sizeof(buffer));

    ::sleep(1);

    uint8_t buffer2[9 + 10*1] = {};

    buffer2[9] = 'a';
    buffer2[10] = 'a';

    transport_node->write(9, buffer2, 10);

    ::sleep(1);

    uint8_t buffer3[9 + 12*1] = {};

    buffer3[9] = 'b';
    buffer3[10] = 'b';

    transport_node->write(12, buffer3, 12);

    return 0;
}
