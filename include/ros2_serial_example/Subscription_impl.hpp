#pragma once

#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <fastcdr/Cdr.h>
#include <fastcdr/FastCdr.h>

#include "ros2_serial_example/subscription.hpp"
#include "ros2_serial_example/transporter.hpp"

template<typename T>
class Subscription_impl : public Subscription
{
public:
    explicit Subscription_impl(const std::shared_ptr<rclcpp::Node> & node,
                               topic_id_size_t mapping,
                               const std::string & name,
                               std::shared_ptr<Transporter> transporter,
                               std::function<size_t(const T &, size_t)> get_size,
                               std::function<bool(const T &, eprosima::fastcdr::Cdr &)> serialize)
    {
        serial_mapping = mapping;
        auto callback = [mapping, transporter, get_size, serialize](const typename T::SharedPtr msg) -> void
        {
            size_t headlen = transporter->get_header_length();
            size_t serialized_size = get_size(*(msg.get()), 0);
            char *data_buffer = new char[headlen + serialized_size];
            eprosima::fastcdr::FastBuffer cdrbuffer(&data_buffer[headlen], serialized_size);
            eprosima::fastcdr::Cdr scdr(cdrbuffer);
            serialize(*(msg.get()), scdr);
            transporter->write(mapping, data_buffer, scdr.getSerializedDataLength());
            delete [] data_buffer;
        };
        sub = node->create_subscription<T>(name, callback, rmw_qos_profile_default);
    }

private:
    std::shared_ptr<rclcpp::Subscription<T>> sub;
};
