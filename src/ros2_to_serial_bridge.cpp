// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Originally based on:
// https://github.com/PX4/px4_ros_com/blob/69bdf6e70f3832ff00f2e9e7f17d9394532787d6/templates/microRTPS_agent.cpp.template
// but modified heavily.

#include <csignal>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include "ros2_serial_example/transporter.hpp"
#include "ros2_serial_example/uart_transporter.hpp"

// Generated file
#include "ros2_topics.hpp"

constexpr int BUFFER_SIZE = 1024;

volatile sig_atomic_t running = 1;

static void signal_handler(int signum)
{
    (void)signum;
    running = 0;
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

static std::unique_ptr<ROS2Topics> parse_node_parameters_for_topics(const std::shared_ptr<rclcpp::Node> & node,
                                                                    std::shared_ptr<Transporter> transporter)
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
            // This is not a parameter in a subsection, so it can't possibly be
            // what we are looking for.  Just silently ignore and continue to
            // allow other parameters.
            continue;
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
            topic_names_and_serialization[topic_name].serial_mapping = static_cast<topic_id_size_t>(serial_mapping);
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

    std::unique_ptr<ROS2Topics> ros2_topics;
    try
    {
        ros2_topics = std::make_unique<ROS2Topics>(node,
                                                   topic_names_and_serialization,
                                                   transporter);
    }
    catch (const std::runtime_error & err)
    {
        ::fprintf(stderr, "%s\n", err.what());
        return nullptr;
    }

    return ros2_topics;
}

void read_thread_func(Transporter * transporter, ROS2Topics * ros2_topics)
{
    char data_buffer[BUFFER_SIZE] = {};
    ssize_t length = 0;
    topic_id_size_t topic_ID;

    while (rclcpp::ok() && running)
    {
        // Process serial -> ROS 2 data
        while ((length = transporter->read(&topic_ID, data_buffer, BUFFER_SIZE)) > 0)
        {
            ros2_topics->dispatch(topic_ID, data_buffer, length);
            topic_ID = std::numeric_limits<topic_id_size_t>::max();
        }
    }
}

int main(int argc, char *argv[])
{
    std::string device{};
    std::string serial_protocol{};
    uint32_t baudrate;

    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("ros2_to_serial_bridge");

    if (!node->get_parameter("device", device))
    {
        ::fprintf(stderr, "No device parameter specified, cannot continue\n");
        return 1;
    }

    if (!node->get_parameter("serial_protocol", serial_protocol))
    {
        ::fprintf(stderr, "No serial_protocol specified, cannot continue\n");
        return 1;
    }

    rclcpp::Parameter baudparm;
    if (node->get_parameter("baudrate", baudparm))
    {
        baudrate = baudparm.get_value<uint32_t>();
    }
    else
    {
        baudrate = 0;
    }

    std::shared_ptr<Transporter> transporter = std::make_shared<UARTTransporter>(device, serial_protocol, baudrate, 100);

    if (transporter->init() < 0)
    {
        return 1;
    }

    std::shared_ptr<ROS2Topics> ros2_topics = parse_node_parameters_for_topics(node, transporter);
    if (ros2_topics == nullptr)
    {
        return 2;
    }

    ::signal(SIGINT, signal_handler);

    std::thread read_thread(read_thread_func, transporter.get(), ros2_topics.get());

    rclcpp::WallRate loop_rate(1000);
    while (rclcpp::ok() && running)
    {
        // Process ROS 2 -> serial data (via callbacks)
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    // This is to handle the case where rclcpp::ok() returned false for some
    // reason; setting running to 0 causes the read_thread to quit as well.
    running = 0;

    read_thread.join();

    transporter->close();

    return 0;
}
