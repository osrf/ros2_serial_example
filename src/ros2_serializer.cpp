#include <csignal>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "ros2_serial_example/ros2_serial_transport.hpp"
#include "ros2_serial_example/ros2_topics.hpp"

constexpr int BUFFER_SIZE = 1024;

volatile sig_atomic_t running = 1;

static void signal_handler(int signum)
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

static void params_usage()
{
    ::printf("Invalid topic parameter.  Topic parameters must be in the form:\n"
             "    topic_name:\n"
             "        serial_mapping: <uint8_t>\n"
             "        type: <string>\n"
             "        direction: [SerialToROS2|ROS2ToSerial]\n"
             );
}

static int parse_options(const std::vector<std::string> & args, std::string & device)
{
    char need_next_arg = ' ';

    // We skip the first element since it contains the program name
    for (auto it = args.begin() + 1; it != args.end(); ++it)
    {
        if ((*it)[0] == '-')
        {
            // This is an option
            // Make sure we aren't waiting for data from a previous option
            if (need_next_arg != ' ')
            {
                usage(args[0].c_str());
                return 1;
            }
            // Make sure it has exactly one more character
            if (it->length() != 2)
            {
                usage(args[0].c_str());
                return 1;
            }

            char opt = (*it)[1];
            switch (opt)
            {
            case 'd':
                need_next_arg = 'd';
                break;
            case 'h':
                usage(args[0].c_str());
                return 0;
            default:
                usage(args[0].c_str());
                return 1;
            };
        }
        else
        {
            if (need_next_arg == ' ')
            {
                // A command-line argument we don't understand
                usage(args[0].c_str());
                return 1;
            }
            else if (need_next_arg == 'd')
            {
                device = *it;
            }
            else
            {
                fprintf(stderr, "Internal error parsing command-line arguments\n");
                return 2;
            }

            need_next_arg = ' ';
        }
    }

    return -1;
}

static std::unique_ptr<ROS2Topics> parse_node_parameters_for_topics(const std::shared_ptr<rclcpp::Node> & node,
                                                                    std::shared_ptr<Transport_node> transport_node)
{
    // Now we go through the YAML file containing our parameters, looking for
    // parameters of the form:
    //     topic_name:
    //         serial_mapping: <uint8_t>
    //         type: <string>
    //         direction: [SerialToROS2|ROS2ToSerial]
    std::map<std::string, TopicMapping> topic_names_and_serialization;

    rcl_interfaces::msg::ListParametersResult list_params_result = node->list_parameters({}, 0);
    for (const auto & name : list_params_result.names)
    {
        if (std::count(name.begin(), name.end(), '.') != 1)
        {
            params_usage();
            return nullptr;
        }
        std::size_t last_dot_pos = name.find_last_of(".");
        if (last_dot_pos == std::string::npos)
        {
            params_usage();
            return nullptr;
        }

        std::string topic_name = name.substr(0, last_dot_pos);
        std::string param_name = name.substr(last_dot_pos + 1);

        if (topic_names_and_serialization.count(topic_name) == 0)
        {
          topic_names_and_serialization[topic_name] = TopicMapping();
        }

        if (param_name == "serial_mapping")
        {
            int64_t serial_mapping = node->get_parameter(name).get_value<int64_t>();
            if (serial_mapping < 1 || serial_mapping > 255)
            {
                params_usage();
                return nullptr;
            }
            topic_names_and_serialization[topic_name].serial_mapping = static_cast<uint8_t>(serial_mapping);
        }
        else if (param_name == "type")
        {
            std::string type = node->get_parameter(name).get_value<std::string>();
            topic_names_and_serialization[topic_name].type = type;
        }
        else if (param_name == "direction")
        {
            std::string dirstring = node->get_parameter(name).get_value<std::string>();
            TopicMapping::Direction direction = TopicMapping::Direction::UNKNOWN;
            if (dirstring == "SerialToROS2")
            {
                direction = TopicMapping::Direction::SERIAL_TO_ROS2;
            }
            else if (dirstring == "ROS2ToSerial")
            {
                direction = TopicMapping::Direction::ROS2_TO_SERIAL;
            }
            else
            {
                params_usage();
                return nullptr;
            }

            topic_names_and_serialization[topic_name].direction = direction;
        }
        else
        {
            params_usage();
            return nullptr;
        }
    }

    std::unique_ptr<ROS2Topics> ros2_topics = std::make_unique<ROS2Topics>();
    if (!ros2_topics->configure(node, topic_names_and_serialization, transport_node))
    {
        return nullptr;
    }

    return ros2_topics;
}

int main(int argc, char *argv[])
{
    std::string device{"/dev/ttyACM0"};

    std::vector<std::string> args = rclcpp::init_and_remove_ros_arguments(argc, argv);

    int ret = parse_options(args, device);
    if (ret >= 0)
    {
        return ret;
    }

    auto node = rclcpp::Node::make_shared("ros2_serializer");

    std::shared_ptr<Transport_node> transport_node = std::make_shared<UART_node>(device.c_str(), B115200, 0);

    if (transport_node->init() < 0)
    {
        return 1;
    }

    std::unique_ptr<ROS2Topics> ros2_topics = parse_node_parameters_for_topics(node, transport_node);
    if (ros2_topics == nullptr)
    {
        return 2;
    }

    ::sleep(1);

    char data_buffer[BUFFER_SIZE] = {};
    ssize_t length = 0;
    uint8_t topic_ID = 255;

    ::signal(SIGINT, signal_handler);

    running = 1;

    while (running)
    {
        while ((length = transport_node->read(&topic_ID, data_buffer, BUFFER_SIZE)) > 0)
        {
            ros2_topics->dispatch(topic_ID, data_buffer, length);
        }
        ::usleep(1);
    }

    transport_node->close();

    return 0;
}
