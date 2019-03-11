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

// Originally based on:
// https://github.com/PX4/px4_ros_com/blob/69bdf6e70f3832ff00f2e9e7f17d9394532787d6/templates/microRTPS_transport.cpp
// but modified to use a ring buffer, fix a few bugs, and split the UART
// implementation to a separate file.

#include <array>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>

#include "ros2_serial_example/transporter.hpp"

namespace ros2_to_serial_bridge
{

namespace transport
{

/** CRC table for the CRC-16. The poly is 0x8005 (x^16 + x^15 + x^2 + 1) */
uint16_t const crc16_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

Transporter::Transporter() : ringbuf(1024)
{
}

Transporter::~Transporter()
{
}

uint16_t Transporter::crc16_byte(uint16_t crc, const uint8_t data)
{
    return (crc >> 8) ^ crc16_table[(crc ^ data) & 0xff];
}

uint16_t Transporter::crc16(uint8_t const *buffer, size_t len)
{
    uint16_t crc = 0;

    while ((len--) != 0)
    {
        crc = crc16_byte(crc, *buffer++);
    }

    return crc;
}

ssize_t Transporter::find_and_copy_message(topic_id_size_t *topic_ID, char out_buffer[], size_t buffer_len)
{
    size_t header_len = get_header_length();
    if (ringbuf.bytes_used() < header_len)
    {
        throw std::runtime_error("Bad size");
    }

    std::array<uint8_t, 3> headerseq{'>', '>', '>'};
    size_t offset = ringbuf.findseq(&headerseq[0], headerseq.size());

    if (offset == ringbuf.bytes_used())
    {
        // We didn't find the sequence; if the ring buffer is full, throw away one
        // bytes to make room for new bytes.
        if (ringbuf.is_full())
        {
            uint8_t dummy;
            ringbuf.memcpy_from(&dummy, 1);
            return 0;
        }
    }

    if (offset > 0)
    {
        // There is some garbage at the front, so just throw it away.
        std::vector<uint8_t> garbage(offset);
        if (ringbuf.memcpy_from(&garbage[0], offset) == nullptr)
        {
            throw std::runtime_error("Failed getting garbage data from ring buffer");
        }
        if (ringbuf.bytes_used() < header_len)
        {
            // Not enough bytes now.
            return 0;
        }
    }

    // Looking for a header of the form:
    // [>,>,>,topic_ID,seq,payload_length_H,payload_length_L,CRCHigh,CRCLow,payloadStart, ... ,payloadEnd]

    uint8_t headerbuf[sizeof(Header)];

    // Peek at the header out of the buffer.  Note that we need to do
    // a peek/copy (rather than just mapping to the array) because the
    // header might be non-contiguous in memory in the ring.

    if (ringbuf.peek(headerbuf, header_len) == nullptr)
    {
        // ringbuf.peek returns nullptr if there isn't enough data in the
        // ring buffer for the requested length
        return 0;
    }

    Header *header = reinterpret_cast<Header *>(headerbuf);

    uint32_t payload_len = (static_cast<uint32_t>(header->payload_len_h) << 8) | header->payload_len_l;

    if (buffer_len < payload_len)
    {
        // The message won't fit the buffer.
        return -EMSGSIZE;
    }

    if (ringbuf.bytes_used() < (header_len + payload_len))
    {
        // We do not have a complete message yet
        return 0;
    }

    // At this point, we know that we have a complete header and the payload.
    // Consume both; whether we keep the data or not, we want it out of the
    // ring.

    // Header
    if (ringbuf.memcpy_from(headerbuf, header_len) == nullptr)
    {
        // We already checked above, so this should never happen.
        throw std::runtime_error("Unexpected ring buffer failure");
    }

    if (ringbuf.memcpy_from(out_buffer, payload_len) == nullptr)
    {
        // We already checked above, so this should never happen.
        throw std::runtime_error("Unexpected ring buffer failure");
    }

    uint16_t read_crc = (static_cast<uint16_t>(header->crc_h) << 8) | header->crc_l;
    uint16_t calc_crc = crc16(reinterpret_cast<uint8_t *>(out_buffer), payload_len);

    ssize_t len;
    if (read_crc != calc_crc)
    {
        ::printf("BAD CRC %u != %u\n", read_crc, calc_crc);
        len = -1;
    }
    else
    {
        *topic_ID = header->topic_ID;
        len = payload_len;
    }

    return len;
}

ssize_t Transporter::read(topic_id_size_t *topic_ID, char out_buffer[], size_t buffer_len)
{
    if (nullptr == out_buffer || nullptr == topic_ID || !fds_OK())
    {
        return -1;
    }

    size_t header_len = get_header_length();

    *topic_ID = std::numeric_limits<topic_id_size_t>::max();

    if (ringbuf.bytes_used() >= header_len)
    {
        ssize_t len = find_and_copy_message(topic_ID, out_buffer, buffer_len);
        if (len > 0)
        {
            return len;
        }
    }

    if (ringbuf.is_full())
    {
        uint8_t dummy;
        ringbuf.memcpy_from(&dummy, 1);
    }

    ssize_t len = node_read();
    if (len <= 0)
    {
        int errsv = errno;

        if (errsv != 0 && EAGAIN != errsv && ETIMEDOUT != errsv)
        {
            ::printf("Read fail %d\n", errsv);
        }

        return len;
    }

    if (ringbuf.bytes_used() >= header_len)
    {
        ssize_t len = find_and_copy_message(topic_ID, out_buffer, buffer_len);
        if (len > 0)
        {
            return len;
        }
    }

    return 0;
}

size_t Transporter::get_header_length()
{
    return sizeof(Header);
}

ssize_t Transporter::write(const topic_id_size_t topic_ID, char buffer[], size_t length)
{
    if (!fds_OK())
    {
        return -1;
    }

    Header header{};
    header.marker[0] = '>';
    header.marker[1] = '>';
    header.marker[2] = '>';

    size_t header_len = get_header_length();

    // [>,>,>,topic_ID,seq,payload_length,CRCHigh,CRCLow,payload_start, ... ,payload_end]

    uint16_t crc = crc16((uint8_t *)&buffer[header_len], length);

    header.topic_ID = topic_ID;
    header.seq = seq++;
    header.payload_len_h = (length >> 8) & 0xff;
    header.payload_len_l = length & 0xff;
    header.crc_h = (crc >> 8) & 0xff;
    header.crc_l = crc & 0xff;

    // Headroom for header is created in client
    // Fill in the header in the same payload buffer to call a single node_write
    ::memcpy(buffer, &header, header_len);

    return node_write(buffer, length + header_len);
}

}  // namespace transport

}  // namespace ros2_to_serial_bridge
