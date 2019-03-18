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
             "    topics:\n"
             "        topic_name:\n"
             "            serial_mapping: <uint8_t>\n"
             "            type: <string>\n"
             "            direction: [SerialToROS2|ROS2ToSerial]\n"
             );
}

static std::unique_ptr<ros2_to_serial_bridge::pubsub::ROS2Topics> parse_node_parameters_for_topics(const std::shared_ptr<rclcpp::Node> & node,
                                                                                                   ros2_to_serial_bridge::transport::Transporter * transporter)
{
    // Now we go through the YAML file containing our parameters, looking for
    // parameters of the form:
    //     topics:
    //         <topic_name>:
    //             serial_mapping: <uint8_t>
    //             type: <string>
    //             direction: [SerialToROS2|ROS2ToSerial]
    std::map<std::string, ros2_to_serial_bridge::pubsub::TopicMapping> topic_names_and_serialization;

    rcl_interfaces::msg::ListParametersResult list_params_result = node->list_parameters({}, 0);
    for (const auto & name : list_params_result.names)
    {
        if (std::count(name.begin(), name.end(), '.') != 2)
        {
            // This is not a parameter in a subsection, so it can't possibly be
            // what we are looking for.  Just silently ignore and continue to
            // allow other parameters.
            continue;
        }

        std::size_t first_dot_pos = name.find_first_of('.');
        if (first_dot_pos == std::string::npos)
        {
            params_usage();
            return nullptr;
        }

        std::string topics = name.substr(0, first_dot_pos);
        if (topics != "topics")
        {
            // This is not a parameter in the subsection topics, so it can't
            // possibly be what we are looking for.  Just silently ignore and
            // continue to look for other parameters.
            continue;
        }

        if (first_dot_pos == name.length())
        {
            // Strangely, the dot is the last character of the parameter.  ROS 2
            // should really never allow this, but just ignore it and continue.
            continue;
        }

        std::size_t second_dot_pos = name.find_first_of('.', first_dot_pos + 1);
        if (second_dot_pos == std::string::npos)
        {
            params_usage();
            return nullptr;
        }

        std::string topic_name = name.substr(first_dot_pos + 1, second_dot_pos - first_dot_pos - 1);

        if (second_dot_pos == name.length())
        {
            // Strangely, the dot is the last character of the parameter.  ROS 2
            // should really never allow this, but just ignore it and continue.
            continue;
        }

        std::string param_name = name.substr(second_dot_pos + 1, name.length() - topic_name.length() - topics.length());

        if (topic_names_and_serialization.count(topic_name) == 0)
        {
            topic_names_and_serialization[topic_name] = ros2_to_serial_bridge::pubsub::TopicMapping();
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
            topic_names_and_serialization[topic_name].type = node->get_parameter(name).get_value<std::string>();
        }
        else if (param_name == "direction")
        {
            std::string dirstring = node->get_parameter(name).get_value<std::string>();
            ros2_to_serial_bridge::pubsub::TopicMapping::Direction direction = ros2_to_serial_bridge::pubsub::TopicMapping::Direction::UNKNOWN;
            if (dirstring == "SerialToROS2")
            {
                direction = ros2_to_serial_bridge::pubsub::TopicMapping::Direction::SERIAL_TO_ROS2;
            }
            else if (dirstring == "ROS2ToSerial")
            {
                direction = ros2_to_serial_bridge::pubsub::TopicMapping::Direction::ROS2_TO_SERIAL;
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

    std::unique_ptr<ros2_to_serial_bridge::pubsub::ROS2Topics> ros2_topics;
    try
    {
        ros2_topics = std::make_unique<ros2_to_serial_bridge::pubsub::ROS2Topics>(node,
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

void read_thread_func(ros2_to_serial_bridge::transport::Transporter * transporter, ros2_to_serial_bridge::pubsub::ROS2Topics * ros2_topics)
{
    // We use a unique_ptr here both to make this a heap allocation and to quiet
    // non-owning pointer warnings from clang-tidy
    std::unique_ptr<uint8_t[]> data_buffer(new uint8_t[BUFFER_SIZE]);
    ssize_t length = 0;
    topic_id_size_t topic_ID;

    while (rclcpp::ok() && running != 0)
    {
        // Process serial -> ROS 2 data
        if ((length = transporter->read(&topic_ID, data_buffer.get(), BUFFER_SIZE)) > 0)
        {
            ros2_topics->dispatch(topic_ID, data_buffer.get(), length);
            topic_ID = std::numeric_limits<topic_id_size_t>::max();
        }
    }
}

int main(int argc, char *argv[])
{
    std::string device{};
    std::string serial_protocol{};
    uint32_t baudrate;
    bool dynamic_serial_mapping{false};

    rclcpp::init(argc, argv);

    // TODO(clalancette): Make this node composable
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

    if (!node->get_parameter("dynamic_serial_mapping", dynamic_serial_mapping))
    {
        ::fprintf(stderr, "No dynamic_serial_mapping specified, cannot continue\n");
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

    std::unique_ptr<ros2_to_serial_bridge::transport::Transporter> transporter = std::make_unique<ros2_to_serial_bridge::transport::UARTTransporter>(device, serial_protocol, baudrate, 100, 8192);

    if (transporter->init() < 0)
    {
        return 1;
    }

    std::unique_ptr<ros2_to_serial_bridge::pubsub::ROS2Topics> ros2_topics;
    if (dynamic_serial_mapping)
    {
        fprintf(stderr, "Dynamic serial mapping not yet implemented\n");
        return 2;
    }
    else
    {
        ros2_topics = parse_node_parameters_for_topics(node, transporter.get());
        if (ros2_topics == nullptr)
        {
            return 2;
        }
    }

    ::signal(SIGINT, signal_handler);

    std::thread read_thread(read_thread_func, transporter.get(), ros2_topics.get());

    // This loop rate translates linearly into the latency to take data from
    // the ROS 2 network and output it to the serial port.  250Hz is a decent
    // balance between CPU time and latency (4ms), but if you are willing to
    // sacrifice CPU time to get better latency, increase this to 1000 or more.
    // It is not recommended to set this lower than 10 (100ms); that will cause
    // the application to feel "sluggish" to the user.
    // TODO(clalancette): Make this configurable via the config file
    rclcpp::WallRate loop_rate(250);
    while (rclcpp::ok() && running != 0)
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

    rclcpp::shutdown();

    return 0;
}