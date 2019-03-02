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
// https://github.com/PX4/px4_ros_com/blob/69bdf6e70f3832ff00f2e9e7f17d9394532787d6/templates/microRTPS_transport.h
// but modified to switch to a ring buffer and to split the UART implementation
// into a separate file.

#pragma once

#include <cstdint>

// If you want to allow > 255 topic name/topic types on the serial wire,
// increase the size of this typedef.  Note that it will break on-wire
// serial compatibility.
typedef uint8_t topic_id_size_t;

class RingBuffer final
{
public:
    explicit RingBuffer(size_t capacity);

    virtual ~RingBuffer();

    ssize_t read(int fd);

    void *peek(void *dst, size_t count);

    void *memcpy_from(void *dst, size_t count);

    size_t findseq(uint8_t *seq, size_t seqlen);

    size_t bytes_used() const;

    bool is_full() const;

private:
    size_t buffer_size() const;
    size_t capacity() const;
    uint8_t *end() const;
    size_t bytes_free() const;
    bool is_empty() const;
    uint8_t *nextp(uint8_t *p);

    uint8_t *buf;
    uint8_t *head;
    uint8_t *tail;
    size_t size;
};

class Transporter
{
public:
    Transporter();
    virtual ~Transporter();

    virtual int init() {return 0;}
    virtual uint8_t close() {return 0;}
    ssize_t read(topic_id_size_t *topic_ID, char out_buffer[], size_t buffer_len);

    /**
     * write a buffer
     * @param topic_ID
     * @param buffer buffer to write: it must leave get_header_length() bytes free at the beginning. This will be
     *               filled with the header. length does not include get_header_length(). So buffer looks like this:
     *                -------------------------------------------------
     *               | header (leave free)          | payload data     |
     *               | get_header_length() bytes    | length bytes     |
     *                -------------------------------------------------
     * @param length buffer length excluding header length
     * @return length on success, <0 on error
     */
    ssize_t write(const topic_id_size_t topic_ID, char buffer[], size_t length);

    /** Get the Length of struct Header to make headroom for the size of struct Header along with payload */
    size_t get_header_length();

protected:
    virtual ssize_t node_read() = 0;
    virtual ssize_t node_write(void *buffer, size_t len) = 0;
    virtual bool fds_OK() = 0;
    uint16_t crc16_byte(uint16_t crc, const uint8_t data);
    uint16_t crc16(uint8_t const *buffer, size_t len);

    RingBuffer ringbuf;

private:
    ssize_t find_and_copy_message(topic_id_size_t *topic_ID, char out_buffer[], size_t buffer_len);

    uint8_t seq{0};
    struct __attribute__((packed)) Header {
        uint8_t marker[3];
        topic_id_size_t topic_ID;
        uint8_t seq;
        uint8_t payload_len_h;
        uint8_t payload_len_l;
        uint8_t crc_h;
        uint8_t crc_l;
    };
};
