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

#ifndef ROS2_SERIAL_EXAMPLE__ROS2_TO_SERIAL_BRIDGE_HPP_
#define ROS2_SERIAL_EXAMPLE__ROS2_TO_SERIAL_BRIDGE_HPP_

#include <future>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "ros2_serial_example/transporter.hpp"

// Generated file
#include "ros2_topics.hpp"

namespace ros2_to_serial_bridge
{

class ROS2ToSerialBridge final : public rclcpp::Node
{
public:
    explicit ROS2ToSerialBridge(const rclcpp::NodeOptions& node_options);
    ROS2ToSerialBridge(ROS2ToSerialBridge const &) = delete;
    ROS2ToSerialBridge& operator=(ROS2ToSerialBridge const &) = delete;
    ROS2ToSerialBridge(ROS2ToSerialBridge &&) = delete;
    ROS2ToSerialBridge& operator=(ROS2ToSerialBridge &&) = delete;

    ~ROS2ToSerialBridge() override;

private:
    void read_thread_func(const std::shared_future<void> & local_future);
    std::map<std::string, ros2_to_serial_bridge::pubsub::TopicMapping> parse_node_parameters_for_topics();
    std::map<std::string, ros2_to_serial_bridge::pubsub::TopicMapping> dynamically_get_serial_mapping(uint64_t wait_ms);

    std::unique_ptr<ros2_to_serial_bridge::transport::Transporter> transporter_;
    std::unique_ptr<ros2_to_serial_bridge::pubsub::ROS2Topics> ros2_topics_;
    std::shared_future<void> future_;
    std::promise<void> exit_signal_;
    std::thread read_thread_;
};

}  // namespace ros2_to_serial_bridge

#endif
