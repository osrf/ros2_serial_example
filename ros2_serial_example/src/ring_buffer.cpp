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
#include <memory>

#include <unistd.h>

#include "ros2_serial_example/ring_buffer.hpp"

namespace ros2_to_serial_bridge
{

namespace transport
{

namespace impl
{

RingBuffer::RingBuffer(size_t capacity) : buf_(std::unique_ptr<uint8_t[]>(new uint8_t[capacity])), size_(capacity)
{
    head_ = tail_ = buf_.get();
}

RingBuffer::~RingBuffer()
{
}

uint8_t *RingBuffer::end() const
{
    return buf_.get() + size_;
}

size_t RingBuffer::bytes_free() const
{
    return size_ - bytes_used();
}

size_t RingBuffer::bytes_used() const
{
    if (full_)
    {
        return size_;
    }

    if (head_ >= tail_)
    {
        return head_ - tail_;
    }

    return size_ + head_ - tail_;
}

bool RingBuffer::is_empty() const
{
    return (!full_ && (head_ == tail_));
}

ssize_t RingBuffer::read(int fd)
{
    uint8_t *bufend = end();
    size_t nfree = bytes_free();

    size_t count = bufend - head_;
    ssize_t n = ::read(fd, head_, count);
    if (n > 0)
    {
        head_ += n;

        // wrap?
        if (head_ == bufend)
        {
            head_ = buf_.get();
        }

        // fix up the tail pointer if an overflow occurred
        if (static_cast<size_t>(n) >= nfree)
        {
            full_ = true;
            tail_ = head_;
        }
    }

    return n;
}

ssize_t RingBuffer::peek(void *dst, size_t count) const
{
    if (count > bytes_used())
    {
        return -1;
    }

    uint8_t *u8dst = static_cast<uint8_t *>(dst);
    uint8_t *bufend = end();
    size_t nwritten = 0;
    uint8_t *tmptail = tail_;
    while (nwritten != count)
    {
        size_t n = std::min(static_cast<size_t>(bufend - tmptail), count - nwritten);
        ::memcpy(u8dst + nwritten, tmptail, n);
        tmptail += n;
        nwritten += n;

        // wrap?
        if (tmptail == bufend)
        {
            tmptail = buf_.get();
        }
    }

    return nwritten;
}

ssize_t RingBuffer::memcpy_from(void *dst, size_t count)
{
    if (dst == nullptr || count == 0 || count > bytes_used())
    {
        return -1;
    }

    uint8_t *u8dst = static_cast<uint8_t *>(dst);
    uint8_t *bufend = end();
    size_t nwritten = 0;
    while (nwritten != count)
    {
        size_t n = std::min(static_cast<size_t>(bufend - tail_), count - nwritten);
        ::memcpy(u8dst + nwritten, tail_, n);
        tail_ += n;
        nwritten += n;

        // wrap?
        if (tail_ == bufend)
        {
            tail_ = buf_.get();
        }
    }

    full_ = false;

    return nwritten;
}

// This returns the number of bytes from tail to the found sequence, or -1 if the sequence can't be found
ssize_t RingBuffer::findseq(const uint8_t *seq, size_t seqlen) const
{
    size_t used = bytes_used();
    if (used < seqlen)
    {
        // There aren't enough bytes in the ring for this sequence, so it
        // can't possibly contain the entire sequence.
        return -1;
    }

    if (seqlen > size_)
    {
        // The sequence to look for is larger than we can possibly hold;
        // this can't work.
        return -1;
    }

    uint8_t *ringp = tail_;
    uint32_t tail_offset{0};
    const uint8_t *seqp = seq;
    uint8_t *bufend = end();

    while (used > 0)
    {
        if (*ringp == *seqp)
        {
            if (seqp == (seq + seqlen - 1))
            {
                // Found it!  Return the offset from tail to the start of the sequence
                return tail_offset - (seqlen - 1);
            }

            seqp++;
        }
        else
        {
            seqp = seq;
        }

        ringp = ringp + 1;
        if (ringp == bufend)
        {
            ringp = buf_.get();
        }
        used--;
        tail_offset++;
    }

    return -1;
}

}  // namespace impl

}  // namespace transport

}  // namespace ros2_to_serial_bridge
