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

#ifndef ROS2_SERIAL_EXAMPLE__ROS2_TOPICS_HPP_
#define ROS2_SERIAL_EXAMPLE__ROS2_TOPICS_HPP_

// C++ includes
#include <algorithm>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// ROS 2 includes
#include <rclcpp/rclcpp.hpp>

#include "ros2_serial_example/transporter.hpp"

@[for t in ros2_types]@
#include "@(t.ns)_@(t.lower_type)_pub_sub_type.hpp"
@[end for]@

namespace ros2_to_serial_bridge
{

namespace pubsub
{

struct TopicMapping final
{
    std::string type{""};
    topic_id_size_t serial_mapping{0};
    enum class Direction
    {
        UNKNOWN,
        SERIAL_TO_ROS2,
        ROS2_TO_SERIAL,
    };
    Direction direction{Direction::UNKNOWN};
};

class ROS2Topics final
{
public:
    explicit ROS2Topics(const std::shared_ptr<rclcpp::Node> & node,
                        const std::map<std::string, TopicMapping> & topic_names_and_serialization,
                        ros2_to_serial_bridge::transport::Transporter * transporter)
    {
        // Setup the pub_type_to_factory map for all types
@[for t in ros2_types]@
        pub_type_to_factory_["@(t.ns)/@(t.ros_type)"] = @(t.ns)_@(t.lower_type)_pub_factory;
@[end for]@

        // Setup the sub_type_to_factory map for all types
@[for t in ros2_types]@
        sub_type_to_factory_["@(t.ns)/@(t.ros_type)"] = @(t.ns)_@(t.lower_type)_sub_factory;
@[end for]@

        // Now go through every topic and ensure that it has a valid type
        // (not ""), a valid serial mapping (not 0), and a valid direction
        // (not UNKNOWN).
        for (const auto & t : topic_names_and_serialization)
        {
            if (t.second.type.empty() || t.second.serial_mapping == 0 || t.second.direction == TopicMapping::Direction::UNKNOWN)
            {
                fprintf(stderr, "Topic '%s' missing type, serial_mapping, or direction; skipping\n", t.first.c_str());
                continue;
            }

            // Check to make sure this isn't a duplicate publication ID.
            if (serial_to_pub_.count(t.second.serial_mapping) != 0)
            {
                throw std::runtime_error("Topic '" + t.first + "' has duplicate pub serial_mapping; this is not allowed");
            }

            // Check to make sure this isn't a duplicate subscription ID.
            if (std::find_if(serial_subs_.begin(), serial_subs_.end(),
                             [t](std::unique_ptr<Subscription> & e) {
                                 return t.second.serial_mapping == e->get_serial_mapping();
                             }) != serial_subs_.end())
            {
                throw std::runtime_error("Topic '" + t.first + "' has duplicate sub serial_mapping; this is not allowed");
            }

            // OK, we've verified that this is a valid topic.  Validate that the
            // direction is valid.
            if (t.second.direction != TopicMapping::Direction::SERIAL_TO_ROS2 && t.second.direction != TopicMapping::Direction::ROS2_TO_SERIAL)
            {
                fprintf(stderr, "Topic '%s' has unsupported direction '%d'; skipping\n", t.first.c_str(), static_cast<int32_t>(t.second.direction));
                continue;
            }

            if (t.second.direction == TopicMapping::Direction::SERIAL_TO_ROS2)
            {
                if (pub_type_to_factory_.count(t.second.type) == 0)
                {
                    fprintf(stderr, "Topic '%s' has unsupported pub type '%s'; skipping\n", t.first.c_str(), t.second.type.c_str());
                    continue;
                }
                serial_to_pub_[t.second.serial_mapping] = pub_type_to_factory_[t.second.type](node, t.first);
            }
            else
            {
                if (sub_type_to_factory_.count(t.second.type) == 0)
                {
                    fprintf(stderr, "Topic '%s' has unsupported sub type '%s'; skipping\n", t.first.c_str(), t.second.type.c_str());
                }
                serial_subs_.push_back(sub_type_to_factory_[t.second.type](node, t.second.serial_mapping, t.first, transporter));
            }
        }
    }

    void dispatch(topic_id_size_t topic_ID, uint8_t *data_buffer, ssize_t length)
    {
        if (serial_to_pub_.count(topic_ID) > 0)
        {
            serial_to_pub_[topic_ID]->dispatch(data_buffer, length);
        }
    }

private:
    std::map<topic_id_size_t, std::unique_ptr<Publisher>> serial_to_pub_;
    std::vector<std::unique_ptr<Subscription>> serial_subs_;
    std::map<std::string, std::function<std::unique_ptr<Publisher>(const std::shared_ptr<rclcpp::Node>, const std::string &)>> pub_type_to_factory_;
    std::map<std::string, std::function<std::unique_ptr<Subscription>(const std::shared_ptr<rclcpp::Node>, topic_id_size_t, const std::string &, ros2_to_serial_bridge::transport::Transporter *)>> sub_type_to_factory_;
};

}  // namespace pubsub
}  // namespace ros2_to_serial_bridge

#endif
