#pragma once

// C++ includes
#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <utility>

// ROS 2 includes
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <fastcdr/Cdr.h>
#include <fastcdr/FastCdr.h>

#include "ros2_serial_example/transporter.hpp"

struct TopicMapping
{
  std::string type{""};
  uint8_t serial_mapping{0};
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
      // Now go through every topic and ensure that it has both a valid type
      // (not "") and a valid serial mapping (not 0).
        for (const auto & t : topic_names_and_serialization)
        {
          if (t.second.type == "" || t.second.serial_mapping == 0 || t.second.direction == TopicMapping::Direction::UNKNOWN)
            {
                return false;
            }

            // OK, we've verified that this is a valid topic.  Let's create the
            // publisher for it.
            if (t.second.type == "std_msgs/String")
            {
                if (t.second.direction == TopicMapping::Direction::SERIAL_TO_ROS2)
                {
                    if (std_msgs_String_serial_to_pub.count(t.second.serial_mapping) != 0)
                    {
                        fprintf(stderr, "Duplicate serial mapping is not allowed\n");
                        return false;
                    }
                    std_msgs_String_serial_to_pub[t.second.serial_mapping] = node->create_publisher<std_msgs::msg::String>(t.first);
                }
                else if (t.second.direction == TopicMapping::Direction::ROS2_TO_SERIAL)
                {
                    auto callback = [transporter](const std_msgs::msg::String::SharedPtr msg) -> void
                    {
                        auto data_buffer = std::make_shared<char>(9 + msg->data.length());
                        eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer.get() + 9, msg->data.length());
                        eprosima::fastcdr::Cdr scdr(cdrbuffer);
                        scdr << msg->data;
                        transporter->write(9, data_buffer.get(), scdr.getSerializedDataLength());
                    };
                    std_msgs_String_serial_subs.push_back(node->create_subscription<std_msgs::msg::String>(t.first, callback));
                    return false;
                }
                else
                {
                    fprintf(stderr, "Unsupported direction\n");
                    return false;
                }
            }
            else
            {
                fprintf(stderr, "Unsupported type\n");
                return false;
            }
        }

        return true;
    }

    void dispatch(uint8_t topic_ID, char data_buffer[], ssize_t length)
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
    std::map<uint8_t, std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>>> std_msgs_String_serial_to_pub;
    std::vector<std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>>> std_msgs_String_serial_subs;
};
