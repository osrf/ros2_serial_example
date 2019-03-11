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

#include <cstdint>

namespace ros2_to_serial_bridge
{

namespace transport
{

namespace impl
{

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

}  // namespace impl

}  // namespace transport

}  // namespace ros2_to_serial_bridge
