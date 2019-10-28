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

#ifndef ROS2_SERIAL_EXAMPLE__SUBSCRIPTION_IMPL_HPP_
#define ROS2_SERIAL_EXAMPLE__SUBSCRIPTION_IMPL_HPP_

#include <cerrno>
#include <cstring>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <fastcdr/Cdr.h>
#include <fastcdr/FastCdr.h>

#include "ros2_serial_example/subscription.hpp"
#include "ros2_serial_example/transporter.hpp"

namespace ros2_to_serial_bridge
{

namespace pubsub
{

/**
 * The SubscriptionImpl class is an implementation of the base Subscription class.
 * As such, it arranges to subscribe to a particular topic on the ROS 2 network.
 * When data arrives on that topic, the callback serializes the data to CDR and
 * then delivers it to the transport for send on the serial port.
 */
template<typename T>
class SubscriptionImpl final : public Subscription
{
public:
    /**
     * Construct a SubscriptionImpl object with the given serialization parameters.
     *
     * @param[in] node The rclcpp::Node to use to create the subscription.
     * @param[in] mapping The number mapping this ROS 2 topic to the serial
     *                    topic ID.
     * @param[in] name The name of the topic to subscribe to.
     * @param[in] transporter A pointer to the transporter object to use to send
     *                        the serialized data to the underlying serial
     *                        transport.
     * @param[in] get_size A function pointer to the function to determine the
     *                     size of the serialization for the CDR type.
     * @param[in] serialize A function pointer to the function to serialize the
     *                      data into CDR.
     */
    explicit SubscriptionImpl(rclcpp::Node * node,
                              topic_id_size_t mapping,
                              const std::string & name,
                              transport::Transporter * transporter,
                              std::function<size_t(const T &, size_t)> get_size,
                              std::function<bool(const T &, eprosima::fastcdr::Cdr &)> serialize) : Subscription()
    {
        serial_mapping_ = mapping;
        auto callback = [node, mapping, transporter, get_size, serialize](const typename T::SharedPtr msg) -> void
        {
            size_t serialized_size = get_size(*(msg.get()), 0);
            std::unique_ptr<uint8_t[]> data_buffer = std::unique_ptr<uint8_t[]>(new uint8_t[serialized_size]{});
            eprosima::fastcdr::FastBuffer cdrbuffer(reinterpret_cast<char *>(data_buffer.get()), serialized_size);
            eprosima::fastcdr::Cdr scdr(cdrbuffer);
            serialize(*(msg.get()), scdr);
            if (transporter->write(mapping, data_buffer.get(), scdr.getSerializedDataLength()) < 0)
            {
                RCLCPP_WARN(node->get_logger(), "Failed to write data: %s", ::strerror(errno));  // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay,hicpp-no-array-decay)
            }
        };
        sub_ = node->create_subscription<T>(name, 10, callback);
    }

private:
    std::shared_ptr<rclcpp::Subscription<T>> sub_;
};

}  // namespace pubsub
}  // namespace ros2_to_serial_bridge

#endif
