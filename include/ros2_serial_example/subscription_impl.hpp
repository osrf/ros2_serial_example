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
            // TODO(clalancette): we aren't checking the return value of write()
            // here, should we?  What would we do on error?
            transporter->write(mapping, data_buffer, scdr.getSerializedDataLength());
            delete [] data_buffer;
        };
        sub = node->create_subscription<T>(name, callback, rmw_qos_profile_default);
    }

private:
    std::shared_ptr<rclcpp::Subscription<T>> sub;
};
