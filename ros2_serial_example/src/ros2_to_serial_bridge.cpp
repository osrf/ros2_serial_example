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

#include <future>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

#include <fastcdr/Cdr.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/detail/empty__rosidl_typesupport_fastrtps_cpp.hpp>

#include "ros2_serial_msgs/msg/serial_mapping.hpp"
#include "ros2_serial_msgs/msg/detail/serial_mapping__rosidl_typesupport_fastrtps_cpp.hpp"

#include "ros2_serial_example/ros2_to_serial_bridge.hpp"
#include "ros2_serial_example/transporter.hpp"
#include "ros2_serial_example/uart_transporter.hpp"
#include "ros2_serial_example/udp_transporter.hpp"

// Generated file
#include "ros2_topics.hpp"

constexpr int BUFFER_SIZE = 1024;

namespace ros2_to_serial_bridge
{
ROS2ToSerialBridge::ROS2ToSerialBridge(const rclcpp::NodeOptions& node_options)
: rclcpp::Node("ros2_to_serial_bridge", rclcpp::NodeOptions(node_options).automatically_declare_parameters_from_overrides(true))
{
    std::string backend_comms{};
    std::string device{};
    std::string backend_protocol{};
    uint32_t baudrate;
    int64_t dynamic_serial_mapping_ms{-1};
    uint32_t read_poll_ms;
    size_t ring_buffer_size;
    uint16_t udp_send_port{0};
    uint16_t udp_recv_port{0};

    if (!get_parameter("backend_comms", backend_comms))
    {
      throw std::runtime_error("No backend comms type specified, cannot continue");
    }

    if (!get_parameter("backend_protocol", backend_protocol))
    {
        throw std::runtime_error("No backend_protocol specified, cannot continue");
    }

    if (!get_parameter("dynamic_serial_mapping_ms", dynamic_serial_mapping_ms))
    {
        throw std::runtime_error("No dynamic_serial_mapping specified, cannot continue");
    }

    if (!get_parameter("read_poll_ms", read_poll_ms))
    {
        throw std::runtime_error("No read_poll_ms specified, cannot continue");
    }

    if (!get_parameter("ring_buffer_size", ring_buffer_size))
    {
        throw std::runtime_error("No ring_buffer_size specified, cannot continue");
    }

    if (backend_comms == "uart")
    {
        if (!get_parameter("device", device))
        {
          throw std::runtime_error("No device parameter specified, cannot continue");
        }

        if (!get_parameter("baudrate", baudrate))
        {
          throw std::runtime_error("No baudrate specified, cannot continue");
        }

        transporter_ = std::make_unique<ros2_to_serial_bridge::transport::UARTTransporter>(device,
                                                                                           backend_protocol,
                                                                                           baudrate,
                                                                                           read_poll_ms,
                                                                                           ring_buffer_size);
    }
    else if (backend_comms == "udp")
    {
        if (!get_parameter("udp_recv_port", udp_recv_port))
        {
          throw std::runtime_error("No udp_recv_port parameter specified, cannot continue");
        }

        if (!get_parameter("udp_send_port", udp_send_port))
        {
          throw std::runtime_error("No udp_send_port parameter specified, cannot continue");
        }

        transporter_ = std::make_unique<ros2_to_serial_bridge::transport::UDPTransporter>(backend_protocol,
                                                                                          udp_recv_port,
                                                                                          udp_send_port,
                                                                                          read_poll_ms,
                                                                                          ring_buffer_size);
    }
    else
    {
        throw std::runtime_error("Invalid backend_comms type specified; only 'uart' and 'udp' are supported");
    }

    if (transporter_->init() < 0)
    {
        throw std::runtime_error("Failed to initialize transport");
    }

    std::map<std::string, ros2_to_serial_bridge::pubsub::TopicMapping> topic_names_and_serialization;
    if (dynamic_serial_mapping_ms >= 0)
    {
        topic_names_and_serialization = dynamically_get_serial_mapping(dynamic_serial_mapping_ms);
    }
    else
    {
        topic_names_and_serialization = parse_node_parameters_for_topics();
    }

    ros2_topics_ = std::make_unique<ros2_to_serial_bridge::pubsub::ROS2Topics>(this,
                                                                               topic_names_and_serialization,
                                                                               transporter_.get());

    future_ = exit_signal_.get_future();

    read_thread_ = std::thread(&ROS2ToSerialBridge::read_thread_func, this, future_);
}

ROS2ToSerialBridge::~ROS2ToSerialBridge()
{
    exit_signal_.set_value();
    read_thread_.join();

    transporter_->close();
}

void ROS2ToSerialBridge::read_thread_func(const std::shared_future<void> & local_future)
{
    std::future_status status;

    // We use a unique_ptr here both to make this a heap allocation and to quiet
    // non-owning pointer warnings from clang-tidy
    std::unique_ptr<uint8_t[]> data_buffer(new uint8_t[BUFFER_SIZE]);
    ssize_t length = 0;
    topic_id_size_t topic_ID;

    do
    {
        // Process serial -> ROS 2 data
        if ((length = transporter_->read(&topic_ID, data_buffer.get(), BUFFER_SIZE)) >= 0)
        {
            ros2_topics_->dispatch(topic_ID, data_buffer.get(), length);
        }
        status = local_future.wait_for(std::chrono::seconds(0));
    } while (status == std::future_status::timeout);
}

std::map<std::string, ros2_to_serial_bridge::pubsub::TopicMapping> ROS2ToSerialBridge::parse_node_parameters_for_topics()
{
    // Now we go through the YAML file containing our parameters, looking for
    // parameters of the form:
    //     topics:
    //         <topic_name>:
    //             serial_mapping: <uint8_t>
    //             type: <string>
    //             direction: [SerialToROS2|ROS2ToSerial]

    std::map<std::string, ros2_to_serial_bridge::pubsub::TopicMapping> topic_names_and_serialization;

    rcl_interfaces::msg::ListParametersResult list_params_result = list_parameters({}, 0);
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
            throw std::runtime_error("Invalid parameter dot position");
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
            throw std::runtime_error("Invalid parameter second dot position");
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
            int64_t serial_mapping = get_parameter(name).get_value<int64_t>();
            topic_names_and_serialization[topic_name].serial_mapping = static_cast<topic_id_size_t>(serial_mapping);
        }
        else if (param_name == "type")
        {
            topic_names_and_serialization[topic_name].type = get_parameter(name).get_value<std::string>();
        }
        else if (param_name == "direction")
        {
            std::string dirstring = get_parameter(name).get_value<std::string>();
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
                throw std::runtime_error("Invalid direction for topic; must be one of 'SerialToROS2' or 'ROS2ToSerial'");
            }

            topic_names_and_serialization[topic_name].direction = direction;
        }
        else
        {
            throw std::runtime_error("Invalid parameter name");
        }
    }

    return topic_names_and_serialization;
}

std::map<std::string, ros2_to_serial_bridge::pubsub::TopicMapping> ROS2ToSerialBridge::dynamically_get_serial_mapping(uint64_t wait_ms)
{
    std::map<std::string, ros2_to_serial_bridge::pubsub::TopicMapping> topic_names_and_serialization;

    {
        // In Crystal and earlier, the std_msgs/Empty message has a zero size.
        // However, that is not true in Dashing and later, so we always attempt
        // to serialize
        std_msgs::msg::Empty dynamic_request;
        size_t serialized_size = std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(dynamic_request, 0);
        std::unique_ptr<uint8_t[]> data_buffer;
        uint8_t *bufferp = nullptr;
        size_t serialized_data_length = 0;
        if (serialized_size != 0)
        {
            data_buffer = std::unique_ptr<uint8_t[]>(new uint8_t[serialized_size]{});
            eprosima::fastcdr::FastBuffer cdrbuffer(reinterpret_cast<char *>(data_buffer.get()), serialized_size);
            eprosima::fastcdr::Cdr scdr(cdrbuffer);
            std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(dynamic_request, scdr);
            bufferp = data_buffer.get();
            serialized_data_length = scdr.getSerializedDataLength();
        }

        if (transporter_->write(0, bufferp, serialized_data_length) < 0)
        {
            throw std::runtime_error("Failed to write dynamic message");
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
        if ((length = transporter_->read(&topic_ID, data_buffer.get(), BUFFER_SIZE)) > 0)
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
                    throw std::runtime_error("Not enough memory for deserialization of SerialMapping message");
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
        throw std::runtime_error("No response to dynamic serial request");
    }

    if (serial_mapping_msg.topic_names.size() != serial_mapping_msg.serial_mappings.size() ||
        serial_mapping_msg.topic_names.size() != serial_mapping_msg.types.size() ||
        serial_mapping_msg.topic_names.size() != serial_mapping_msg.direction.size())
    {
        throw std::runtime_error("Serial mapping message names, mappings, types, and directions must all be the same size");
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
            throw std::runtime_error("Unknown direction for topic, cannot continue");
        }
    }

    return topic_names_and_serialization;
}

}  // namespace ros2_to_serial_bridge

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_to_serial_bridge::ROS2ToSerialBridge)
