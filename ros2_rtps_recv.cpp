#include <csignal>
#include <cstdio>
#include <cstring>
#include <memory>

#include <termios.h>
#include <unistd.h>

#include "RtpsTopics.hpp"
#include "ros2_serial_transport.hpp"

volatile sig_atomic_t running = 1;

void signal_handler(int signum)
{
    (void)signum;
    running = 0;
}

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
          if (optarg != nullptr) {
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

    ::signal(SIGINT, signal_handler);

    if (transport_node->init() < 0)
    {
        return 1;
    }

    RtpsTopics topics;

    topics.init();

    while (running)
    {
        uint8_t topic_ID = 255;
        while (topics.hasMsg(&topic_ID))
        {
        }

        ::usleep(1);
    }

    transport_node->close();

    return 0;
}
