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

#ifndef ROS2_SERIAL_EXAMPLE__PUBLISHER_HPP_
#define ROS2_SERIAL_EXAMPLE__PUBLISHER_HPP_

#include <cstdint>

#include <sys/types.h>

namespace ros2_to_serial_bridge
{

namespace pubsub
{

/**
 * The Publisher class provides an abstract class for taking CDR-serialized data
 * in a buffer and publishing it on the ROS 2 network.  Derived classes should
 * override the pure virtual dispatch() method to actually do the work to
 * publish the message on the ROS 2 network.
 */
class Publisher
{
public:
    /**
     * Do publisher-specific initialization.
     *
     * Since this is an abstract class, it can't be directly constructed, but
     * this constructor is expected to be called during the derived class
     * constructor to setup the Publisher.
     */
    Publisher() {};
    virtual ~Publisher() {};

    Publisher(Publisher const &) = delete;
    Publisher& operator=(Publisher const &) = delete;
    Publisher(Publisher &&) = delete;
    Publisher& operator=(Publisher &&) = delete;

    /**
     * Pure virtual method for taking CDR-serialized data and sending it to the ROS 2 network.
     *
     * Derived classes should override this method and implement the code for
     * CDR deserialization and send on the ROS 2 network.
     *
     * @param[in] data_buffer The buffer containing the CDR-serialized data to send
     * @param[in] length The length of the data_buffer
     */
    virtual void dispatch(uint8_t *data_buffer, ssize_t length) = 0;
};

}  // namespace pubsub
}  // namespace ros2_to_serial_bridge

#endif
