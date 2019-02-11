#include <csignal>
#include <cstdio>
#include <cstring>
#include <memory>

#include <unistd.h>

#include "RtpsTopics.hpp"

constexpr int BUFFER_SIZE = 1024;

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
    int ch;
    while ((ch = ::getopt(argc, argv, "h")) != EOF)
    {
      switch (ch)
      {
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

    ::signal(SIGINT, signal_handler);

    RtpsTopics topics;

    topics.init_subs();

    while (running)
    {
        uint8_t topic_ID = 255;
        while (topics.hasMsg(&topic_ID))
        {
            // uint16_t header_length = transport_node->get_header_length();
            uint16_t header_length = 9;
            char data_buffer[BUFFER_SIZE] = {};
            /* make room for the header to fill in later */
            eprosima::fastcdr::FastBuffer cdrbuffer(&data_buffer[header_length], sizeof(data_buffer)-header_length);
            eprosima::fastcdr::Cdr scdr(cdrbuffer);
            if (topics.getMsg(topic_ID, scdr))
            {
                int length = scdr.getSerializedDataLength();
                fprintf(stderr, "Got a message of length %d\n", length);
            }
        }

        ::usleep(1);
    }

    return 0;
}
