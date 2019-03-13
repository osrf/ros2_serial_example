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

#ifndef ROS2_SERIAL_EXAMPLE__PUBLISHER_IMPL_HPP_
#define ROS2_SERIAL_EXAMPLE__PUBLISHER_IMPL_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <fastcdr/Cdr.h>
#include <fastcdr/FastCdr.h>

#include "ros2_serial_example/publisher.hpp"

namespace ros2_to_serial_bridge
{

namespace pubsub
{

template<typename T>
class PublisherImpl final : public Publisher
{
public:
    explicit PublisherImpl(const std::shared_ptr<rclcpp::Node> & node, const std::string & name,
                           std::function<bool(eprosima::fastcdr::Cdr &, T &)> des) : deserialize(des)
    {
        pub = node->create_publisher<T>(name);
    }

    void dispatch(uint8_t *data_buffer, ssize_t length) override
    {
        eprosima::fastcdr::FastBuffer cdrbuffer(reinterpret_cast<char *>(data_buffer), length);
        eprosima::fastcdr::Cdr cdrdes(cdrbuffer);
        auto msg = std::make_shared<T>();
        deserialize(cdrdes, *(msg.get()));
        pub->publish(msg);
    }

private:
    std::function<bool(eprosima::fastcdr::Cdr &, T &)> deserialize;
    std::shared_ptr<rclcpp::Publisher<T>> pub;
};

}  // namespace pubsub
}  // namespace ros2_to_serial_bridge

#endif
