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

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <fastcdr/Cdr.h>
#include <fastcdr/FastCdr.h>

#include <@(ros2_type.ns)/msg/@(ros2_type.lower_type).hpp>
#include <@(ros2_type.ns)/msg/detail/@(ros2_type.lower_type)__rosidl_typesupport_fastrtps_cpp.hpp>

#include "@(ros2_type.ns)_@(ros2_type.lower_type)_pub_sub_type.hpp"

#include "ros2_serial_example/publisher.hpp"
#include "ros2_serial_example/publisher_impl.hpp"
#include "ros2_serial_example/subscription.hpp"
#include "ros2_serial_example/subscription_impl.hpp"

namespace ros2_to_serial_bridge
{

namespace pubsub
{

std::unique_ptr<Publisher> @(ros2_type.ns)_@(ros2_type.lower_type)_pub_factory(rclcpp::Node * node, const std::string & topic)
{
    typedef bool (*des_t)(eprosima::fastcdr::Cdr &, @(ros2_type.ns)::msg::@(ros2_type.ros_type) &);
    des_t des = @(ros2_type.ns)::msg::typesupport_fastrtps_cpp::cdr_deserialize;
    return std::make_unique<PublisherImpl<@(ros2_type.ns)::msg::@(ros2_type.ros_type)>>(node, topic, des);
}

std::unique_ptr<Subscription> @(ros2_type.ns)_@(ros2_type.lower_type)_sub_factory(rclcpp::Node * node, topic_id_size_t serial_mapping, const std::string & topic, ros2_to_serial_bridge::transport::Transporter * transporter)
{
    typedef size_t (*getsize_t)(const @(ros2_type.ns)::msg::@(ros2_type.ros_type) &, size_t);
    getsize_t getsize = @(ros2_type.ns)::msg::typesupport_fastrtps_cpp::get_serialized_size;
    typedef bool (*ser_t)(const @(ros2_type.ns)::msg::@(ros2_type.ros_type) &, eprosima::fastcdr::Cdr &);
    ser_t ser = @(ros2_type.ns)::msg::typesupport_fastrtps_cpp::cdr_serialize;

    return std::make_unique<SubscriptionImpl<@(ros2_type.ns)::msg::@(ros2_type.ros_type)>>(node, serial_mapping, topic, transporter, getsize, ser);
}

}  // namespace pubsub
}  // namespace ros2_to_serial_bridge
