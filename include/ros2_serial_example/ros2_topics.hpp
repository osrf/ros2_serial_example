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

class ROS2Topics
{

public:
    ROS2Topics()
    {
    }

    bool configure(const std::shared_ptr<rclcpp::Node> & node,
                   const std::map<std::string, std::pair<std::string, uint8_t>> & topic_names_and_serialization)
    {
      // Now go through every topic and ensure that it has both a valid type
      // (not "") and a valid serial mapping (not 0).
        for (const auto & t : topic_names_and_serialization)
        {
            if (t.second.first == "" || t.second.second == 0)
            {
                return false;
            }

            // OK, we've verified that this is a valid topic.  Let's create the
            // publisher for it.
            if (t.second.first == "std_msgs/String")
            {
                if (std_msgs_String_serial_to_pub.count(t.second.second) != 0)
                {
                    fprintf(stderr, "Duplicate serial mapping is not allowed\n");
                    return false;
                }
                std_msgs_String_serial_to_pub[t.second.second] = node->create_publisher<std_msgs::msg::String>(t.first);
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
};
