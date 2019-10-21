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
#include <stdexcept>

#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include "ros2_serial_example/ros2_to_serial_bridge.hpp"

volatile sig_atomic_t running = 1;

static void signal_handler(int signum)
{
    (void)signum;
    running = 0;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    ::signal(SIGINT, signal_handler);

    std::shared_ptr<ros2_to_serial_bridge::ROS2ToSerialBridge> node;
    try
    {
        rclcpp::NodeOptions node_options;
        node_options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
        node = std::make_shared<ros2_to_serial_bridge::ROS2ToSerialBridge>(node_options);
    }
    catch (const std::runtime_error & err)
    {
        ::fprintf(stderr, "Failed to construct node: %s\n", err.what());
        return 1;
    }

    uint64_t write_sleep_ms;
    if (!node->get_parameter("write_sleep_ms", write_sleep_ms))
    {
        throw std::runtime_error("No write_sleep_ms specified, cannot continue");
    }

    rclcpp::WallRate loop_rate(1000.0 / static_cast<double>(write_sleep_ms));
    while (rclcpp::ok() && running != 0)
    {
        // Process ROS 2 -> serial data (via callbacks)
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    // This is to handle the case where rclcpp::ok() returned false for some
    // reason; setting running to 0 causes the read_thread to quit as well.
    running = 0;

    rclcpp::shutdown();

    return 0;
}
