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

// C++ includes
#include <algorithm>
#include <cstdint>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// ROS 2 includes
#include <rclcpp/rclcpp.hpp>

#include "ros2_serial_example/transporter.hpp"

#include "ros2_serial_example/publisher.hpp"
#include "ros2_serial_example/subscription.hpp"
#include "ros2_serial_example/publisher_impl.hpp"
#include "ros2_serial_example/subscription_impl.hpp"

@[for t in types]@
#include <@(t.include)>
@[end for]@

struct TopicMapping
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

class ROS2Topics
{

public:
    explicit ROS2Topics(const std::shared_ptr<rclcpp::Node> & node,
                        const std::map<std::string, TopicMapping> & topic_names_and_serialization,
                        std::shared_ptr<Transporter> transporter)
    {
        // Now go through every topic and ensure that it has a valid type
        // (not ""), a valid serial mapping (not 0), and a valid direction
        // (not UNKNOWN).
        for (const auto & t : topic_names_and_serialization)
        {
            if (t.second.type == "" || t.second.serial_mapping == 0 || t.second.direction == TopicMapping::Direction::UNKNOWN)
            {
                fprintf(stderr, "Topic '%s' missing type, serial_mapping, or direction; skipping\n", t.first.c_str());
                continue;
            }

            // Check to make sure this isn't a duplicate publication ID.
            if (serial_to_pub.count(t.second.serial_mapping) != 0)
            {
                throw std::runtime_error("Topic '" + t.first + "' has duplicate pub serial_mapping; this is not allowed");
            }

            // Check to make sure this isn't a duplicate subscription ID.
            if (std::find_if(serial_subs.begin(), serial_subs.end(),
                             [t](std::unique_ptr<Subscription> & e) {
                                 return t.second.serial_mapping == e->get_serial_mapping();
                             }) != serial_subs.end())
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

@[for i,t in enumerate(types)]@
            @(i != 0 ? "else ")if (t.second.type == "@(t.name)")
            {
                if (t.second.direction == TopicMapping::Direction::SERIAL_TO_ROS2)
                {
                    // We have to do this dance to with the function types
                    // because the compiler cannot automatically figure out the
                    // correct overload.
                    typedef bool (*des_t)(eprosima::fastcdr::Cdr &, @(t.cpp_type) &);
                    des_t func = @(t.serialize_ns)::cdr_deserialize;
                    serial_to_pub[t.second.serial_mapping] = std::make_unique<Publisher_impl<@(t.cpp_type)>>(node, t.first, func);
                }
                else
                {
                    // We have to do this dance to with the function types
                    // because the compiler cannot automatically figure out the
                    // correct overload.
                    typedef size_t (*getsize_t)(const @(t.cpp_type) &, size_t);
                    getsize_t getsize = @(t.serialize_ns)::get_serialized_size;
                    typedef bool (*ser_t)(const @(t.cpp_type) &, eprosima::fastcdr::Cdr &);
                    ser_t ser = @(t.serialize_ns)::cdr_serialize;
                    serial_subs.push_back(std::make_unique<Subscription_impl<@(t.cpp_type)>>(node, t.second.serial_mapping, t.first, transporter, getsize, ser));
                }
            }
@[end for]@
            else
            {
                fprintf(stderr, "Topic '%s' has unsupported type '%s'; skipping\n", t.first.c_str(), t.second.type.c_str());
                continue;
            }
        }
    }

    void dispatch(topic_id_size_t topic_ID, char data_buffer[], ssize_t length)
    {
        if (serial_to_pub.count(topic_ID) > 0)
        {
            serial_to_pub[topic_ID]->dispatch(data_buffer, length);
        }
    }

private:
    std::map<topic_id_size_t, std::unique_ptr<Publisher>> serial_to_pub;
    std::vector<std::unique_ptr<Subscription>> serial_subs;
};
