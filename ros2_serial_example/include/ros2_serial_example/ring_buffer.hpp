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

#ifndef ROS2_SERIAL_EXAMPLE__RING_BUFFER_HPP_
#define ROS2_SERIAL_EXAMPLE__RING_BUFFER_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>

namespace ros2_to_serial_bridge
{

namespace transport
{

namespace impl
{

/**
 * The RingBuffer class provides a C++-like API on top of a ring buffer
 * implementation.
 *
 * The general idea behind the ring buffer is that there are two pointers,
 * 'head' and 'tail'.  New data coming in is added at the head pointer, and the
 * head pointer is then updated.  If the number of incoming bytes exceeds the
 * size of the ring, then the 'full' flag is set.  Data being removed from the
 * buffer is read from the tail pointer, and the tail pointer is then updated.
 * If the full flag is set, and head == tail, the ring buffer is full.  If the
 * full flag is not set, and head == tail, the ring buffer is empty.  If the
 * ring buffer becomes full, and new data is attempted to be added, the oldest
 * data in the ring is overwritten and lost.
 *
 * This implementation only expects a single reader and writer at a time, so
 * multi-threaded programs should protect accesses with a mutex.
 *
 * One semi-unique feature of this implementation is the read() method, which
 * does a POSIX read directly into the read buffer.
 */
class RingBuffer
{
public:
    explicit RingBuffer(size_t capacity);

    virtual ~RingBuffer();

    RingBuffer(RingBuffer const &) = delete;
    RingBuffer& operator=(RingBuffer const &) = delete;
    RingBuffer(RingBuffer &&) = delete;
    RingBuffer& operator=(RingBuffer &&) = delete;

    /**
     * Read data from a file descriptor directly into the ring buffer.
     *
     * This improves performance by skipping a read() followed by a copy into the
     * ring buffer.
     *
     * @param[in] fd The file descriptor to read data out of.
     * @returns The number of bytes read on success, or -1 on error.
     *
     * @note This method can and does do short reads, either because the
     *       underlying file descriptor returned a short read or because the
     *       internal mechanics of the ring buffer implementation forced it to.
     *       Callers should be prepared to deal with short reads.
     *
     * @note This method expects that the file descriptor passed in is
     *       non-blocking and returns immediately if there is no data available.
     *       Passing a blocking file descriptor here could cause this method
     *       to block for long periods of time.
     */
    ssize_t read(int fd);

    /**
     * Look at the data currently in the ring buffer without removing it.
     *
     * The ring buffer is not altered.  This call can fail if the ring buffer
     * doesn't contain at least the number of bytes requested.
     *
     * @param[out] dst The destination buffer for the data; this should be
     *                 at least as large as count.
     * @param[in] count The size of data to look for in the ring buffer.
     * @returns The number of bytes copied out on success, or -1 on error.
     */
    ssize_t peek(void *dst, size_t count) const;

    /**
     * Copy data out of the ring buffer into a linear buffer.
     *
     * If there isn't enough data in the buffer for the number of bytes
     * requested, or the number of bytes requested is 0, or the destination
     * is a nullptr, this returns -1.  Otherwise it removes the requested
     * number of bytes from the ring.
     *
     * @param[out] dst The destination buffer for the data; this should be
     *                 at least as large as count.
     * @param[in] count The size of the data to copy out of the ring buffer.
     * @returns The number of bytes copied out on success, or -1 on error.
     */
    ssize_t memcpy_from(void *dst, size_t count);

    /**
     * Find a particular sequence of bytes in the ring buffer.
     *
     * If the sequence of bytes is found, then the offset from the 'tail' of
     * the ring is returned.  This is the number of bytes that would have to be
     * copied out of the ring to get to the sequence.  If the sequence cannot
     * be found for any reason, -1 is returned.
     *
     * @param[in] seq The byte sequence to look for in the ring; this should be
     *                at least as large as seqlen.
     * @param[in] seqlen The length of the byte sequence to look for.
     * @returns The offset from the tail of the ring on success, -1 on error.
     */
    ssize_t findseq(const uint8_t *seq, size_t seqlen) const;

    /**
     * Get the number of valid bytes the ring buffer is using.
     *
     * @returns The number of valid bytes the ring buffer is using.
     */
    size_t bytes_used() const;

protected:
    /**
     * Get a pointer to the end of the ring buffer.
     *
     * @returns a pointer to the end of the ring buffer.
     */
    uint8_t *end() const;

    /**
     * Get the number of bytes free in the ring buffer.
     *
     * @returns the number of bytes free in the ring buffer.
     */
    size_t bytes_free() const;

    /**
     * Determine whether the ring buffer is empty.
     *
     * @returns true if the ring buffer is empty, false otherwise.
     */
    bool is_empty() const;

    std::unique_ptr<uint8_t[]> buf_;
    uint8_t *head_;
    uint8_t *tail_;
    bool full_{false};
    size_t size_;
};

}  // namespace impl
}  // namespace transport
}  // namespace ros2_to_serial_bridge

#endif
