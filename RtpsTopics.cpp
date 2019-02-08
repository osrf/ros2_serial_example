/****************************************************************************
 *
 * Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <cstddef>
#include <cstdint>

#include "RtpsTopics.hpp"

bool RtpsTopics::init()
{
    // Initialise publishers
    if (_battery_status_pub.init()) {
        std::cout << "battery_status publisher start" << std::endl;
    } else {
        std::cout << "ERROR starting battery_status publisher" << std::endl;
        return false;
    }
// @[for topic in send_topics]@
//     if (_@(topic)_pub.init()) {
//         std::cout << "@(topic) publisher started" << std::endl;
//     } else {
//         std::cout << "ERROR starting @(topic) publisher" << std::endl;
//         return false;
//     }

// @[end for]@
    return true;
}

void RtpsTopics::publish(uint8_t topic_ID, char data_buffer[], size_t len)
{
    switch (topic_ID)
    {
        case 6: // battery_status
        {
            battery_status_ st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            _battery_status_pub.publish(&st);
        }
        break;

// @[for topic in send_topics]@
//         case @(rtps_message_id(ids, topic)): // @(topic)
//         {
//             @(package[0])::msg::dds_::@(topic)_ st;
//             eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
//             eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
//             st.deserialize(cdr_des);
//             _@(topic)_pub.publish(&st);
//         }
//         break;
// @[end for]@
      default:
        break;
    }
}
