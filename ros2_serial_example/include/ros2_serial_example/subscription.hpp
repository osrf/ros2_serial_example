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

#ifndef ROS2_SERIAL_EXAMPLE__SUBSCRIPTION_HPP_
#define ROS2_SERIAL_EXAMPLE__SUBSCRIPTION_HPP_

#include "ros2_serial_example/transporter.hpp"

namespace ros2_to_serial_bridge
{

namespace pubsub
{

/**
 * The Subscription class provides an base class for taking data from the ROS 2
 * network and serializing it to CDR to be sent to the serial transport.
 */
class Subscription
{
public:
    /**
     * Do subscription-specific initialization.
     *
     * Since this is an abstract class, it can't be directly constructed, but
     * this constructor is expected to be called during the derived class
     * constructor to setup the Publisher.
     */
    Subscription() {}
    virtual ~Subscription() {}

    Subscription(Subscription const &) = delete;
    Subscription& operator=(Subscription const &) = delete;
    Subscription(Subscription &&) = delete;
    Subscription& operator=(Subscription &&) = delete;

    /**
     * Get the serial mapping number for this subscription.
     *
     * @returns The serial mapping number for this subscription.
     */
    topic_id_size_t get_serial_mapping() const
    {
        return serial_mapping_;
    }

protected:
    topic_id_size_t serial_mapping_{0};
};

}  // namespace pubsub
}  // namespace ros2_to_serial_bridge

#endif
