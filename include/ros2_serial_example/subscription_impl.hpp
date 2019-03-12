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

#pragma once

#include <cstring>
#include <functional>
#include <memory>
#include <string>

#include <errno.h>

#include <rclcpp/rclcpp.hpp>

#include <fastcdr/Cdr.h>
#include <fastcdr/FastCdr.h>

#include "ros2_serial_example/subscription.hpp"
#include "ros2_serial_example/transporter.hpp"

namespace ros2_to_serial_bridge
{

namespace pubsub
{

template<typename T>
class Subscription_impl final : public Subscription
{
public:
    explicit Subscription_impl(const std::shared_ptr<rclcpp::Node> & node,
                               topic_id_size_t mapping,
                               const std::string & name,
                               std::shared_ptr<transport::Transporter> transporter,
                               std::function<size_t(const T &, size_t)> get_size,
                               std::function<bool(const T &, eprosima::fastcdr::Cdr &)> serialize)
    {
        size_t headlen = transporter->get_header_length();
        serial_mapping = mapping;
        auto callback = [node, mapping, transporter, get_size, serialize, headlen](const typename T::SharedPtr msg) -> void
        {
            size_t serialized_size = get_size(*(msg.get()), 0);
            std::unique_ptr<char[]> data_buffer = std::unique_ptr<char[]>(new char[headlen + serialized_size]);
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer.get() + headlen, serialized_size);
            eprosima::fastcdr::Cdr scdr(cdrbuffer);
            serialize(*(msg.get()), scdr);
            if (transporter->write(mapping, data_buffer.get(), scdr.getSerializedDataLength()) < 0)
            {
                RCLCPP_WARN(node->get_logger(), "Failed to write data: %s", ::strerror(errno));
            }
        };
        sub = node->create_subscription<T>(name, callback, rmw_qos_profile_default);
    }

private:
    std::shared_ptr<rclcpp::Subscription<T>> sub;
};

}  // namespace pubsub
}  // namespace ros2_to_serial_bridge
