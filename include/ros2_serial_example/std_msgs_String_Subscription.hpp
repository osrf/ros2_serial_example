#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/string__rosidl_typesupport_fastrtps_cpp.hpp>

#include <fastcdr/Cdr.h>
#include <fastcdr/FastCdr.h>

#include "ros2_serial_example/subscription.hpp"
#include "ros2_serial_example/transporter.hpp"

class std_msgs_String_Subscription : public Subscription
{
public:
    explicit std_msgs_String_Subscription(const std::shared_ptr<rclcpp::Node> & node,
                                          topic_id_size_t mapping,
                                          const std::string & name,
                                          std::shared_ptr<Transporter> transporter)
    {
        serial_mapping = mapping;
        auto callback = [mapping, transporter](const std_msgs::msg::String::SharedPtr msg) -> void
        {
            size_t headlen = transporter->get_header_length();
            size_t serialized_size = std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(*(msg.get()), 0);
            char *data_buffer = new char[headlen + serialized_size];
            eprosima::fastcdr::FastBuffer cdrbuffer(&data_buffer[headlen], serialized_size);
            eprosima::fastcdr::Cdr scdr(cdrbuffer);
            std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(*(msg.get()), scdr);
            transporter->write(mapping, data_buffer, scdr.getSerializedDataLength());
            delete [] data_buffer;
        };
        sub = node->create_subscription<std_msgs::msg::String>(name, callback, rmw_qos_profile_default);
    }

private:
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> sub;
};
