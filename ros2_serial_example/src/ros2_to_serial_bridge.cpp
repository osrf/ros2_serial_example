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

#include <cerrno>
#include <chrono>
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
#include <std_msgs/msg/empty__rosidl_typesupport_fastrtps_cpp.hpp>

#include "ros2_serial_msgs/msg/serial_mapping.hpp"
#include "ros2_serial_msgs/msg/serial_mapping__rosidl_typesupport_fastrtps_cpp.hpp"

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

static int parse_node_parameters_for_topics(const std::shared_ptr<rclcpp::Node> & node,
                                            std::map<std::string, ros2_to_serial_bridge::pubsub::TopicMapping> & topic_names_and_serialization)
{
    // Now we go through the YAML file containing our parameters, looking for
    // parameters of the form:
    //     topics:
    //         <topic_name>:
    //             serial_mapping: <uint8_t>
    //             type: <string>
    //             direction: [SerialToROS2|ROS2ToSerial]

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
            return -1;
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
            return -1;
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
                return -1;
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
                return -1;
            }

            topic_names_and_serialization[topic_name].direction = direction;
        }
        else
        {
            params_usage();
            return -1;
        }
    }

    return 0;
}

static int dynamically_get_serial_mapping(ros2_to_serial_bridge::transport::Transporter * transporter,
                                          uint64_t wait_ms,
                                          std::map<std::string, ros2_to_serial_bridge::pubsub::TopicMapping> & topic_names_and_serialization)
{
    {
        std_msgs::msg::Empty dynamic_request;
        size_t serialized_size = std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(dynamic_request, 0);
        std::unique_ptr<uint8_t[]> data_buffer = std::unique_ptr<uint8_t[]>(new uint8_t[serialized_size]{});
        eprosima::fastcdr::FastBuffer cdrbuffer(reinterpret_cast<char *>(data_buffer.get()), serialized_size);
        eprosima::fastcdr::Cdr scdr(cdrbuffer);
        std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(dynamic_request, scdr);
        if (transporter->write(0, data_buffer.get(), scdr.getSerializedDataLength()) < 0)
        {
            ::fprintf(stderr, "Failed to write dynamic message: %s\n", ::strerror(errno));
            return -1;
        }
    }

    // Wait for up to wait_ms for a response
    std::unique_ptr<uint8_t[]> data_buffer(new uint8_t[BUFFER_SIZE]);
    std::chrono::duration<uint64_t, std::ratio<1, 1000>> diff_ms{0};
    bool got_response{false};
    ros2_serial_msgs::msg::SerialMapping serial_mapping_msg;
    std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
    do
    {
        ssize_t length = 0;
        topic_id_size_t topic_ID;
        if ((length = transporter->read(&topic_ID, data_buffer.get(), BUFFER_SIZE)) > 0)
        {
            if (topic_ID == 1)
            {
                eprosima::fastcdr::FastBuffer cdrbuffer(reinterpret_cast<char *>(data_buffer.get()), length);
                eprosima::fastcdr::Cdr cdrdes(cdrbuffer);
                // Deserialization can fail if the message isn't actually
                // a SerialMapping message, in which case Fast-CDR will
                // throw eprosima::fastcdr::exception::NotEnoughMemoryException.
                try
                {
                    ros2_serial_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(cdrdes, serial_mapping_msg);
                }
                catch(const eprosima::fastcdr::exception::NotEnoughMemoryException & err)
                {
                    ::fprintf(stderr, "Not enough memory for deserialization of SerialMapping message\n");
                    return -1;
                }
                got_response = true;
                break;
              }
          }

        if (wait_ms > 0)
        {
            std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
            diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
        }
    } while (diff_ms.count() < wait_ms);

    if (!got_response)
    {
        ::fprintf(stderr, "No response to dynamic serial request\n");
        return -1;
    }

    if (serial_mapping_msg.topic_names.size() != serial_mapping_msg.serial_mappings.size() ||
        serial_mapping_msg.topic_names.size() != serial_mapping_msg.types.size() ||
        serial_mapping_msg.topic_names.size() != serial_mapping_msg.direction.size())
    {
        ::fprintf(stderr, "Serial mapping message names, mappings, types, and directions must all be the same size\n");
        return -1;
    }

    for (size_t i = 0; i < serial_mapping_msg.topic_names.size(); ++i)
    {
        std::string topic_name = serial_mapping_msg.topic_names[i];

        topic_names_and_serialization[topic_name] = ros2_to_serial_bridge::pubsub::TopicMapping();
        topic_names_and_serialization[topic_name].serial_mapping = serial_mapping_msg.serial_mappings[i];
        topic_names_and_serialization[topic_name].type = serial_mapping_msg.types[i];

        uint8_t direction = serial_mapping_msg.direction[i];
        if (direction == ros2_serial_msgs::msg::SerialMapping::SERIALTOROS2)
        {
            topic_names_and_serialization[topic_name].direction = ros2_to_serial_bridge::pubsub::TopicMapping::Direction::SERIAL_TO_ROS2;
        }
        else if (direction == ros2_serial_msgs::msg::SerialMapping::ROS2TOSERIAL)
        {
            topic_names_and_serialization[topic_name].direction = ros2_to_serial_bridge::pubsub::TopicMapping::Direction::ROS2_TO_SERIAL;
        }
        else
        {
            fprintf(stderr, "Unknown direction for topic, cannot continue\n");
            return -1;
        }
    }

    return 0;
}

void read_thread_func(ros2_to_serial_bridge::transport::Transporter * transporter,
                      ros2_to_serial_bridge::pubsub::ROS2Topics * ros2_topics)
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
    int64_t dynamic_serial_mapping_ms{-1};
    uint32_t read_poll_ms;
    size_t ring_buffer_size;
    uint64_t write_sleep_ms;

    rclcpp::init(argc, argv);

    // TODO(clalancette): Make this node composable
    rclcpp::Node::SharedPtr node;
    try
    {
        node = rclcpp::Node::make_shared("ros2_to_serial_bridge");
    }
    catch (const std::runtime_error & err)
    {
        ::fprintf(stderr, "Failed to construct node: %s\n", err.what());
        return 1;
    }

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

    if (!node->get_parameter("dynamic_serial_mapping_ms", dynamic_serial_mapping_ms))
    {
        ::fprintf(stderr, "No dynamic_serial_mapping specified, cannot continue\n");
        return 1;
    }

    if (!node->get_parameter("baudrate", baudrate))
    {
        ::fprintf(stderr, "No baudrate specified, cannot continue\n");
        return 1;
    }

    if (!node->get_parameter("read_poll_ms", read_poll_ms))
    {
        ::fprintf(stderr, "No read_poll_ms specified, cannot continue\n");
        return 1;
    }

    if (!node->get_parameter("ring_buffer_size", ring_buffer_size))
    {
        ::fprintf(stderr, "No ring_buffer_size specified, cannot continue\n");
        return 1;
    }

    if (!node->get_parameter("write_sleep_ms", write_sleep_ms))
    {
        ::fprintf(stderr, "No write_sleep_ms specified, cannot continue\n");
        return 1;
    }

    std::unique_ptr<ros2_to_serial_bridge::transport::Transporter> transporter = std::make_unique<ros2_to_serial_bridge::transport::UARTTransporter>(device, serial_protocol, baudrate, read_poll_ms, ring_buffer_size);

    if (transporter->init() < 0)
    {
        return 1;
    }

    std::map<std::string, ros2_to_serial_bridge::pubsub::TopicMapping> topic_names_and_serialization;
    if (dynamic_serial_mapping_ms > 0)
    {
        if (dynamically_get_serial_mapping(transporter.get(), dynamic_serial_mapping_ms, topic_names_and_serialization) < 0)
        {
            return 2;
        }
    }
    else
    {
        if (parse_node_parameters_for_topics(node, topic_names_and_serialization) < 0)
        {
            return 2;
        }
    }

    std::unique_ptr<ros2_to_serial_bridge::pubsub::ROS2Topics> ros2_topics;
    try
    {
        ros2_topics = std::make_unique<ros2_to_serial_bridge::pubsub::ROS2Topics>(node,
                                                                                  topic_names_and_serialization,
                                                                                  transporter.get());
    }
    catch (const std::runtime_error & err)
    {
        ::fprintf(stderr, "%s\n", err.what());
        return 2;
    }

    ::signal(SIGINT, signal_handler);

    std::thread read_thread(read_thread_func, transporter.get(), ros2_topics.get());

    rclcpp::WallRate loop_rate(1000.0 / static_cast<double>(write_sleep_ms));
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
