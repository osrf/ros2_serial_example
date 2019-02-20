#pragma once

// C++ includes
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

// ROS 2 includes
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <fastcdr/Cdr.h>
#include <fastcdr/FastCdr.h>

#include "ros2_serial_example/transporter.hpp"

struct TopicMapping
{
    std::string type{""};
    topic_id_size_t serial_mapping{0};
    enum class Direction
    {
        UNKNOWN,
        SERIAL_TO_ROS2,
        ROS2_TO_SERIAL,
    };
    Direction direction{Direction::UNKNOWN};
};

class ROS2Topics
{

public:
    ROS2Topics()
    {
    }

    bool configure(const std::shared_ptr<rclcpp::Node> & node,
                   const std::map<std::string, TopicMapping> & topic_names_and_serialization,
                   std::shared_ptr<Transporter> transporter)
    {
        // Now go through every topic and ensure that it has a valid type
        // (not ""), a valid serial mapping (not 0), and a valid direction
        // (not UNKNOWN).
        for (const auto & t : topic_names_and_serialization)
        {
            if (t.second.type == "" || t.second.serial_mapping == 0 || t.second.direction == TopicMapping::Direction::UNKNOWN)
            {
                fprintf(stderr, "Topic '%s' missing type, serial_mapping, or direction; skipping\n", t.first.c_str());
                continue;
            }

            // OK, we've verified that this is a valid topic.  Let's create the
            // publisher for it.
            if (t.second.type == "std_msgs/String")
            {
                if (t.second.direction == TopicMapping::Direction::SERIAL_TO_ROS2)
                {
                    if (std_msgs_String_serial_to_pub.count(t.second.serial_mapping) != 0)
                    {
                        fprintf(stderr, "Topic '%s' has duplicate serial_mapping; this is not allowed\n", t.first.c_str());
                        return false;
                    }
                    std_msgs_String_serial_to_pub[t.second.serial_mapping] = node->create_publisher<std_msgs::msg::String>(t.first);
                }
                else if (t.second.direction == TopicMapping::Direction::ROS2_TO_SERIAL)
                {
                    auto callback = [t, transporter](const std_msgs::msg::String::SharedPtr msg) -> void
                    {
                        size_t headlen = transporter->get_header_length();
                        char *data_buffer = new char[msg->data.size() + 5 + headlen]();
                        eprosima::fastcdr::FastBuffer cdrbuffer(&data_buffer[headlen], msg->data.size() + 5);
                        eprosima::fastcdr::Cdr scdr(cdrbuffer);
                        scdr << msg->data;
                        transporter->write(t.second.serial_mapping, data_buffer, scdr.getSerializedDataLength());
                        delete [] data_buffer;
                    };
                    std_msgs_String_serial_subs.push_back(node->create_subscription<std_msgs::msg::String>(t.first, callback, rmw_qos_profile_default));
                }
                else
                {
                    fprintf(stderr, "Topic '%s' has unsupported direction '%d'; skipping\n", t.first.c_str(), static_cast<int32_t>(t.second.direction));
                    continue;
                }
            }
            else
            {
                fprintf(stderr, "Topic '%s' has unsupported type '%s'; skipping\n", t.first.c_str(), t.second.type.c_str());
                continue;
            }
        }

        return true;
    }

    void dispatch(topic_id_size_t topic_ID, char data_buffer[], ssize_t length)
    {
        if (std_msgs_String_serial_to_pub.count(topic_ID) > 0)
        {
              // string topic
              eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, length);
              eprosima::fastcdr::Cdr cdrdes(cdrbuffer);
              auto msg = std::make_shared<std_msgs::msg::String>();
              cdrdes >> msg->data;
              std_msgs_String_serial_to_pub[topic_ID]->publish(msg);
        }
    }

private:
    std::map<topic_id_size_t, std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>>> std_msgs_String_serial_to_pub;
    std::vector<std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>>> std_msgs_String_serial_subs;
};
