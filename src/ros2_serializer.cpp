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

#include "diux/ros2_serial_transport.hpp"

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
             "  topic_name:\n"
             "    serial_mapping: <uint8_t>\n"
             "    type: <string>\n"
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

static int parse_node_parameters_for_topics(const std::shared_ptr<rclcpp::Node> & node,
                                            std::map<uint8_t, std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>>> & string_serial_to_pub)
{
    // Now we go through the YAML file containing our parameters, looking for
    // parameters of the form:
    //     topic_name:
    //         serial_mapping: <uint8_t>
    //         type: <string>
    std::map<std::string, std::pair<std::string, uint8_t>> topic_names_and_serialization;

    rcl_interfaces::msg::ListParametersResult list_params_result = node->list_parameters({}, 0);
    for (const auto & name : list_params_result.names)
    {
        if (std::count(name.begin(), name.end(), '.') != 1)
        {
            params_usage();
            return 3;
        }
        std::size_t last_dot_pos = name.find_last_of(".");
        if (last_dot_pos == std::string::npos)
        {
            params_usage();
            return 3;
        }

        std::string topic_name = name.substr(0, last_dot_pos);
        std::string param_name = name.substr(last_dot_pos + 1);
        if (param_name == "serial_mapping")
        {
            int64_t serial_mapping = node->get_parameter(name).get_value<int64_t>();
            if (serial_mapping < 1 || serial_mapping > 255)
            {
                params_usage();
                return 3;
            }
            if (topic_names_and_serialization.count(topic_name) == 0)
            {
                topic_names_and_serialization[topic_name] = std::make_pair("", static_cast<uint8_t>(serial_mapping));
            }
            else
            {
                topic_names_and_serialization[topic_name].second = static_cast<uint8_t>(serial_mapping);
            }
        }
        else if (param_name == "type")
        {
            std::string type = node->get_parameter(name).get_value<std::string>();
            if (topic_names_and_serialization.count(topic_name) == 0)
            {
                topic_names_and_serialization[topic_name] = std::make_pair(type, 0);
            }
            else
            {
                topic_names_and_serialization[topic_name].first = type;
            }
        }
        else
        {
            params_usage();
            return 3;
        }
    }

    // Now go through every topic and ensure that it has both a valid type
    // (not "") and a valid serial mapping (not 0).
    for (const auto & t : topic_names_and_serialization)
    {
        if (t.second.first == "" || t.second.second == 0)
        {
            params_usage();
            return 3;
        }

        // OK, we've verified that this is a valid topic.  Let's create the
        // publisher for it.
        if (t.second.first == "std_msgs/String")
        {
            if (string_serial_to_pub.count(t.second.second) != 0)
            {
                fprintf(stderr, "Duplicate serial mapping is not allowed\n");
                return 5;
            }
            string_serial_to_pub[t.second.second] = node->create_publisher<std_msgs::msg::String>(t.first);
        }
        else
        {
            fprintf(stderr, "Unsupported type\n");
            return 4;
        }
    }

    return -1;
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

    std::map<uint8_t, std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>>> string_serial_to_pub;
    ret = parse_node_parameters_for_topics(node, string_serial_to_pub);
    if (ret >= 0)
    {
        return ret;
    }

    std::unique_ptr<Transport_node> transport_node = std::make_unique<UART_node>(device.c_str(), B115200, 0);

    if (transport_node->init() < 0)
    {
        return 1;
    }

    ::sleep(1);

    uint8_t data_buffer[BUFFER_SIZE] = {};
    int length = 0;
    uint8_t topic_ID = 255;

    ::signal(SIGINT, signal_handler);

    running = 1;

    while (running)
    {
        while ((length = transport_node->read(&topic_ID, data_buffer, BUFFER_SIZE)) > 0)
        {
            if (string_serial_to_pub.count(topic_ID) > 0)
            {
                  // string topic
                  auto msg = std::make_shared<std_msgs::msg::String>();
                  char buf[2];
                  buf[0] = data_buffer[0];
                  buf[1] = '\0';
                  msg->data = std::string(buf);
                  string_serial_to_pub[topic_ID]->publish(msg);
            }
        }
        ::usleep(1);
    }

    transport_node->close();

    return 0;
}
