#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/string__rosidl_typesupport_fastrtps_cpp.hpp>

#include <fastcdr/Cdr.h>
#include <fastcdr/FastCdr.h>

#include "ros2_serial_example/publisher.hpp"

class std_msgs_String_Publisher : public Publisher
{
public:
    explicit std_msgs_String_Publisher(const std::shared_ptr<rclcpp::Node> & node, const std::string & name)
    {
        pub = node->create_publisher<std_msgs::msg::String>(name);
    }

    void dispatch(char data_buffer[], ssize_t length) override
    {
        // string topic
        eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, length);
        eprosima::fastcdr::Cdr cdrdes(cdrbuffer);
        auto msg = std::make_shared<std_msgs::msg::String>();
        std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(cdrdes, *(msg.get()));
        pub->publish(msg);
    }

private:
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> pub;
};
