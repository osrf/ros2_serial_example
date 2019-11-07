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

/**
 * The PublisherImpl class is an implementation of the abstract Publisher class.
 * As such, it implements the pure virtual function dispatch() to take
 * CDR-serialized data and transmit it on the ROS 2 network.  One of these
 * objects will be created for each SerialToROS2 topic mapping the user sets up
 * in the bridge configuration file.
 */
template<typename T>
class PublisherImpl final : public Publisher
{
public:
    /**
     * Construct a PublisherImpl object with the given serialization parameters.
     *
     * @param[in] node The rclcpp::Node to use to create a publisher.
     * @param[in] name The name of the topic to publish to.
     * @param[in] des A function pointer to the deserialization function for
     *                this CDR type.
     */
    explicit PublisherImpl(rclcpp::Node * node, const std::string & name,
                           std::function<bool(eprosima::fastcdr::Cdr &, T &)> des)
        : deserialize_(des), name_(name), node_(node)
    {
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        pub_ = node_->create_publisher<T>(name, qos);
    }

    /**
     * Implementation of method to take CDR-serialized data and send it to the ROS 2 network.
     *
     * @param[in] data_buffer The buffer containing the CDR-serialized data to send
     * @param[in] length The length of the data_buffer
     */
    void dispatch(uint8_t *data_buffer, ssize_t length) override
    {
        eprosima::fastcdr::FastBuffer cdrbuffer(reinterpret_cast<char *>(data_buffer), length);
        eprosima::fastcdr::Cdr cdrdes(cdrbuffer);
        auto msg = std::make_unique<T>();
        // Deserialization can fail if, for instance, the user told us the
        // wrong type to deserialize (they configured it as a std_msgs/String
        // when it is actually a std_msgs/UInt16, for instance).  In that case
        // Fast-CDR will throw an
        // eprosima::fastcdr::exception::NotEnoughMemoryException, so handle it
        // here and print a message.
        try
        {
            deserialize_(cdrdes, *(msg.get()));
        }
        catch(const eprosima::fastcdr::exception::NotEnoughMemoryException & err)
        {
            RCLCPP_WARN(node_->get_logger(),  // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay,hicpp-no-array-decay)
                        "Not enough memory for deserialization on topic '%s'; is the type correct?",
                        name_.c_str());
            return;
        }
        pub_->publish(std::move(msg));
    }

private:
    std::function<bool(eprosima::fastcdr::Cdr &, T &)> deserialize_;
    std::string name_;
    rclcpp::Node * node_;
    std::shared_ptr<rclcpp::Publisher<T>> pub_;
};

}  // namespace pubsub
}  // namespace ros2_to_serial_bridge

#endif
