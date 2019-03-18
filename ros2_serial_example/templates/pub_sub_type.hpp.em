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

#ifndef ROS2_SERIAL_EXAMPLE__@(ros2_type.ns.upper())_@(ros2_type.lower_type.upper())_HPP_
#define ROS2_SERIAL_EXAMPLE__@(ros2_type.ns.upper())_@(ros2_type.lower_type.upper())_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "ros2_serial_example/publisher.hpp"
#include "ros2_serial_example/subscription.hpp"
#include "ros2_serial_example/transporter.hpp"

namespace ros2_to_serial_bridge
{

namespace pubsub
{

std::unique_ptr<Publisher> @(ros2_type.ns)_@(ros2_type.lower_type)_pub_factory(rclcpp::Node * node, const std::string & topic);
std::unique_ptr<Subscription> @(ros2_type.ns)_@(ros2_type.lower_type)_sub_factory(rclcpp::Node * node, topic_id_size_t serial_mapping, const std::string & topic, ros2_to_serial_bridge::transport::Transporter * transporter);

}  // namespace pubsub
}  // namespace ros2_to_serial_bridge

#endif
