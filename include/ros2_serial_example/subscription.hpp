#pragma once

#include "ros2_serial_example/transporter.hpp"

class Subscription
{
public:
    topic_id_size_t get_serial_mapping() const
    {
        return serial_mapping;
    }

protected:
    topic_id_size_t serial_mapping;
};
