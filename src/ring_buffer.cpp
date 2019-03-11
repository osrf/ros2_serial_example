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

#include <algorithm>
#include <cstdint>
#include <cstring>

#include <unistd.h>

#include "ros2_serial_example/ring_buffer.hpp"

namespace ros2_to_serial_bridge
{

namespace transport
{

namespace impl
{

RingBuffer::RingBuffer(size_t capacity)
{
    size = capacity + 1;
    buf = new uint8_t[size];
    // TODO(clalancette): check for nullptr
    head = tail = buf;
}

RingBuffer::~RingBuffer()
{
    delete [] buf;
}

size_t RingBuffer::buffer_size() const
{
    return size;
}

size_t RingBuffer::capacity() const
{
    return size - 1;
}

uint8_t *RingBuffer::end() const
{
    return buf + size;
}

size_t RingBuffer::bytes_free() const
{
    if (head >= tail)
    {
        return capacity() - (head - tail);
    }

    return tail - head - 1;
}

size_t RingBuffer::bytes_used() const
{
    return capacity() - bytes_free();
}

bool RingBuffer::is_full() const
{
    return bytes_free() == 0;
}

bool RingBuffer::is_empty() const
{
    return bytes_free() == capacity();
}

uint8_t *RingBuffer::nextp(uint8_t *p)
{
    // Given a ring buffer rb and a pointer to a location within its
    // contiguous buffer, return the a pointer to the next logical
    // location in the ring buffer.
    return buf + ((++p - buf) % buffer_size());
}

ssize_t RingBuffer::read(int fd)
{
    uint8_t *bufend = end();
    size_t nfree = bytes_free();

    size_t count = bufend - head;
    ssize_t n = ::read(fd, head, count);
    if (n > 0)
    {
        head += n;

        // wrap?
        if (head == bufend)
        {
            head = buf;
        }

        // fix up the tail pointer if an overflow occurred
        if (static_cast<size_t>(n) > nfree)
        {
            tail = nextp(head);
        }
    }

    return n;
}

void *RingBuffer::peek(void *dst, size_t count)
{
    if (count > bytes_used())
    {
        return nullptr;
    }

    uint8_t *u8dst = static_cast<uint8_t *>(dst);
    uint8_t *bufend = end();
    size_t nwritten = 0;
    uint8_t *tmptail = tail;
    while (nwritten != count)
    {
        size_t n = std::min(static_cast<size_t>(bufend - tmptail), count - nwritten);
        ::memcpy(u8dst + nwritten, tmptail, n);
        tmptail += n;
        nwritten += n;

        // wrap?
        if (tmptail == bufend)
        {
            tmptail = buf;
        }
    }

    return tmptail;
}

void *RingBuffer::memcpy_from(void *dst, size_t count)
{
    if (count > bytes_used())
    {
        return nullptr;
    }

    uint8_t *u8dst = static_cast<uint8_t *>(dst);
    uint8_t *bufend = end();
    size_t nwritten = 0;
    while (nwritten != count)
    {
        size_t n = std::min(static_cast<size_t>(bufend - tail), count - nwritten);
        ::memcpy(u8dst + nwritten, tail, n);
        tail += n;
        nwritten += n;

        // wrap?
        if (tail == bufend)
        {
          tail = buf;
        }
    }

    return tail;
}

size_t RingBuffer::findseq(uint8_t *seq, size_t seqlen)
{
    size_t used = bytes_used();
    if (used < seqlen)
    {
        // There aren't enough bytes in the ring for this sequence, so it
        // can't possibly contain the entire sequence.
        return used;
    }

    if (seqlen > capacity())
    {
        // The sequence to look for is larger than we can possibly hold;
        // this can't work.
        return used;
    }

    uint8_t *ringp = tail;
    uint8_t *seqp = seq;
    uint8_t *found = nullptr;
    uint8_t *start = buf + ((tail - buf) % buffer_size());

    while (ringp != head)
    {
        if (*ringp == *seqp)
        {
            if (found != nullptr && seqp == (seq + seqlen - 1))
            {
                // Found it!  Return the start of the sequence
                return found - start;
            }

            if (found == nullptr)
            {
                found = ringp;
            }
            seqp++;
        }
        else
        {
            found = nullptr;
        }
        ringp = nextp(ringp);
    }

    return used;
}

}  // namespace impl

}  // namespace transport

}  // namespace ros2_to_serial_bridge
