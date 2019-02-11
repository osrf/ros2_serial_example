#include <csignal>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>

#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "ros2_serial_transport.hpp"

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
    char device[64] = "/dev/ttyACM0";

    rclcpp::init(argc, argv);

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

    ::sleep(1);

    auto node = rclcpp::Node::make_shared("ros2_serializer");
    auto str_pub = node->create_publisher<std_msgs::msg::String>("chatter");

    char data_buffer[BUFFER_SIZE] = {};
    int length = 0;
    uint8_t topic_ID = 255;

    ::signal(SIGINT, signal_handler);

    running = 1;

    while (running)
    {
        while ((length = transport_node->read(&topic_ID, data_buffer, BUFFER_SIZE)) > 0)
        {
            if (topic_ID == 9)
            {
                // string topic
                auto msg = std::make_shared<std_msgs::msg::String>();
                char buf[2];
                buf[0] = data_buffer[0];
                buf[1] = '\0';
                msg->data = std::string(buf);
                str_pub->publish(msg);
            }
        }
        ::usleep(1);
    }

    transport_node->close();

    return 0;
}
