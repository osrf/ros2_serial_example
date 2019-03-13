#include <gtest/gtest.h>

#include <linux/memfd.h>

#include <fcntl.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <unistd.h>

#include "ros2_serial_example/ring_buffer.hpp"

/// HELPERS

static inline int memfd_create(const char *name, unsigned int flags)
{
    return syscall(__NR_memfd_create, name, flags);
}

int setup_memfd(const std::string & name)
{
    int fd = memfd_create(name.c_str(), MFD_CLOEXEC);
    if (fd < 0)
    {
        return -1;
    }

    // The RingBuffer class essentially assumes that this file descriptor
    // is non-blocking
    if (::fcntl(fd, F_SETFL, O_NONBLOCK) != 0)
    {
        ::close(fd);
        return -1;
    }

    return fd;
}

// Add some additional data to the memfd.  After this call the file pointer will
// be at the start of the new memory.
int add_to_memfd(int fd, uint8_t *buf, size_t bufsize)
{
    off_t offset = ::lseek(fd, 0, SEEK_CUR);
    if (offset < 0)
    {
        return -1;
    }

    if (::ftruncate(fd, offset + bufsize) != 0)
    {
        return -1;
    }

    if (::lseek(fd, offset, SEEK_SET) != offset)
    {
        return -1;
    }

    if (::write(fd, buf, bufsize) != static_cast<int>(bufsize))
    {
        return -1;
    }

    if (::lseek(fd, offset, SEEK_SET) != offset)
    {
        return -1;
    }

    return bufsize;
}

// This fixture allows us access to the protected methods of RingBuffer
class RingBufferFixture : public ros2_to_serial_bridge::transport::impl::RingBuffer, public testing::Test
{
public:
    // We pick a size of 240 bytes for the ring buffer to ensure that we can fill
    // uint8_t into the buffer for the tests
    RingBufferFixture() : RingBuffer(240) {}
};

/// TESTS

TEST_F(RingBufferFixture, empty)
{
    ASSERT_EQ(bytes_used(), 0U);
    ASSERT_FALSE(is_full());
    ASSERT_EQ(size_, 240U);
    ASSERT_EQ(bytes_free(), size_);
    ASSERT_TRUE(is_empty());
    ASSERT_EQ(head_, buf_.get());
    ASSERT_EQ(tail_, buf_.get());
}

TEST_F(RingBufferFixture, read)
{
    int fd = setup_memfd("ringbuffer_read_test");
    ASSERT_GE(fd, 0);

    uint8_t data[3]{0x0, 0x1, 0x2};
    ASSERT_EQ(add_to_memfd(fd, data, sizeof(data)), static_cast<int>(sizeof(data)));

    ASSERT_EQ(read(fd), static_cast<ssize_t>(sizeof(data)));

    ASSERT_EQ(bytes_used(), sizeof(data));
    ASSERT_FALSE(is_full());
    ASSERT_EQ(size_, 240U);
    ASSERT_EQ(bytes_free(), size_ - sizeof(data));
    ASSERT_FALSE(is_empty());
    ASSERT_EQ(head_, buf_.get() + sizeof(data));
    ASSERT_EQ(tail_, buf_.get());
    ASSERT_FALSE(full_);
    uint8_t *bufp = buf_.get();
    ASSERT_EQ(*bufp++, 0x0);
    ASSERT_EQ(*bufp++, 0x1);
    ASSERT_EQ(*bufp++, 0x2);

    ::close(fd);
}

TEST_F(RingBufferFixture, read_fill)
{
    int fd = setup_memfd("ringbuffer_readwrap_test");
    ASSERT_GE(fd, 0);

    // Start by filling up *most* of the buffer
    uint8_t initialbuf[238];
    for (uint8_t i = 0; i < sizeof(initialbuf); ++i)
    {
        initialbuf[i] = i;
    }
    ASSERT_EQ(add_to_memfd(fd, initialbuf, sizeof(initialbuf)), static_cast<int>(sizeof(initialbuf)));

    ASSERT_EQ(read(fd), static_cast<ssize_t>(sizeof(initialbuf)));

    ASSERT_EQ(bytes_used(), sizeof(initialbuf));
    ASSERT_FALSE(is_full());
    ASSERT_EQ(size_, 240U);
    ASSERT_EQ(bytes_free(), size_ - sizeof(initialbuf));
    ASSERT_FALSE(is_empty());
    ASSERT_EQ(head_, buf_.get() + sizeof(initialbuf));
    ASSERT_EQ(tail_, buf_.get());
    ASSERT_FALSE(full_);

    // Now exactly fill it
    uint8_t smallbuf[2]{238, 239};
    ASSERT_EQ(add_to_memfd(fd, smallbuf, sizeof(smallbuf)), static_cast<int>(sizeof(smallbuf)));

    ASSERT_EQ(read(fd), static_cast<ssize_t>(sizeof(smallbuf)));

    ASSERT_EQ(bytes_used(), sizeof(initialbuf) + sizeof(smallbuf));
    ASSERT_TRUE(is_full());
    ASSERT_EQ(size_, 240U);
    ASSERT_EQ(bytes_free(), size_ - sizeof(initialbuf) - sizeof(smallbuf));
    ASSERT_FALSE(is_empty());
    ASSERT_TRUE(full_);

    uint8_t *bufp = buf_.get();
    for (uint8_t i = 0; i < size_; ++i)
    {
        ASSERT_EQ(*bufp, i);
        bufp++;
    }

    ::close(fd);
}

TEST_F(RingBufferFixture, read_wrap)
{
    int fd = setup_memfd("ringbuffer_readwrap_test");
    ASSERT_GE(fd, 0);

    // Start by filling up the entire buffer
    uint8_t initialbuf[240];
    for (uint8_t i = 0; i < sizeof(initialbuf); ++i)
    {
        initialbuf[i] = i;
    }
    ASSERT_EQ(add_to_memfd(fd, initialbuf, sizeof(initialbuf)), static_cast<int>(sizeof(initialbuf)));

    ASSERT_EQ(read(fd), static_cast<ssize_t>(sizeof(initialbuf)));

    ASSERT_EQ(bytes_used(), sizeof(initialbuf));
    ASSERT_TRUE(is_full());
    ASSERT_EQ(size_, 240U);
    ASSERT_EQ(bytes_free(), size_ - sizeof(initialbuf));
    ASSERT_FALSE(is_empty());
    ASSERT_EQ(head_, buf_.get());
    ASSERT_EQ(tail_, buf_.get());
    ASSERT_TRUE(full_);

    uint8_t *bufp = buf_.get();
    for (uint8_t i = 0; i < size_; ++i)
    {
        ASSERT_EQ(*bufp, i);
        bufp++;
    }

    // Now overflow
    uint8_t smallbuf[2]{240, 241};
    ASSERT_EQ(add_to_memfd(fd, smallbuf, sizeof(smallbuf)), static_cast<int>(sizeof(smallbuf)));

    ASSERT_EQ(read(fd), static_cast<ssize_t>(sizeof(smallbuf)));

    ASSERT_EQ(bytes_used(), size_);
    ASSERT_TRUE(is_full());
    ASSERT_EQ(size_, 240U);
    ASSERT_EQ(bytes_free(), 0U);
    ASSERT_FALSE(is_empty());
    ASSERT_EQ(head_, buf_.get() + sizeof(smallbuf));
    ASSERT_EQ(tail_, buf_.get() + sizeof(smallbuf));
    ASSERT_TRUE(full_);

    bufp = buf_.get();
    ASSERT_EQ(*bufp++, 240);
    ASSERT_EQ(*bufp++, 241);
    for (uint8_t i = 2; i < size_; ++i)
    {
        ASSERT_EQ(*bufp, i);
        bufp++;
    }

    ::close(fd);
}

TEST_F(RingBufferFixture, memcpy_from_nullptr)
{
    ASSERT_EQ(memcpy_from(nullptr, 10), -1);
}

TEST_F(RingBufferFixture, memcpy_from_0_count)
{
    uint8_t cpybuf[10]{};
    ASSERT_EQ(memcpy_from(cpybuf, 0), -1);
}

TEST_F(RingBufferFixture, memcpy_from_not_enough_bytes)
{
    // Try to memcpy out 10 bytes
    uint8_t cpybuf[10]{};
    ASSERT_EQ(memcpy_from(cpybuf, sizeof(cpybuf)), -1);
}

TEST_F(RingBufferFixture, memcpy_from_start_buffer)
{
    int fd = setup_memfd("ringbuffer_read_test");
    ASSERT_GE(fd, 0);

    uint8_t data[3]{0x0, 0x1, 0x2};
    ASSERT_EQ(add_to_memfd(fd, data, sizeof(data)), static_cast<int>(sizeof(data)));

    ASSERT_EQ(read(fd), static_cast<ssize_t>(sizeof(data)));

    ASSERT_EQ(bytes_used(), sizeof(data));
    ASSERT_FALSE(is_full());
    ASSERT_EQ(size_, 240U);
    ASSERT_EQ(bytes_free(), size_ - sizeof(data));
    ASSERT_FALSE(is_empty());
    ASSERT_EQ(head_, buf_.get() + sizeof(data));
    ASSERT_EQ(tail_, buf_.get());
    ASSERT_FALSE(full_);

    uint8_t cpybuf[3]{};
    ASSERT_EQ(memcpy_from(cpybuf, sizeof(cpybuf)), static_cast<ssize_t>(sizeof(cpybuf)));
    ASSERT_EQ(cpybuf[0], 0x0);
    ASSERT_EQ(cpybuf[1], 0x1);
    ASSERT_EQ(cpybuf[2], 0x2);

    ASSERT_EQ(bytes_used(), 0U);
    ASSERT_FALSE(is_full());
    ASSERT_EQ(size_, 240U);
    ASSERT_EQ(bytes_free(), size_);
    ASSERT_TRUE(is_empty());
    ASSERT_EQ(head_, buf_.get() + sizeof(data));
    ASSERT_EQ(tail_, buf_.get() + sizeof(data));
    ASSERT_FALSE(full_);

    ::close(fd);
}

TEST_F(RingBufferFixture, memcpy_from_full_buffer)
{
    int fd = setup_memfd("ringbuffer_read_test");
    ASSERT_GE(fd, 0);

    // Start by filling up the entire buffer
    uint8_t initialbuf[240];
    for (uint8_t i = 0; i < sizeof(initialbuf); ++i)
    {
        initialbuf[i] = i;
    }
    ASSERT_EQ(add_to_memfd(fd, initialbuf, sizeof(initialbuf)), static_cast<int>(sizeof(initialbuf)));

    ASSERT_EQ(read(fd), static_cast<ssize_t>(sizeof(initialbuf)));

    ASSERT_EQ(bytes_used(), sizeof(initialbuf));
    ASSERT_TRUE(is_full());
    ASSERT_EQ(size_, 240U);
    ASSERT_EQ(bytes_free(), size_ - sizeof(initialbuf));
    ASSERT_FALSE(is_empty());
    ASSERT_EQ(head_, buf_.get());
    ASSERT_EQ(tail_, buf_.get());
    ASSERT_TRUE(full_);

    for (uint8_t i = 0; i < size_; ++i)
    {
        ASSERT_EQ(buf_[i], i);
    }

    // Now copy a bit of data out
    uint8_t cpybuf[3]{};
    ASSERT_EQ(memcpy_from(cpybuf, sizeof(cpybuf)), static_cast<ssize_t>(sizeof(cpybuf)));
    ASSERT_EQ(cpybuf[0], 0x0);
    ASSERT_EQ(cpybuf[1], 0x1);
    ASSERT_EQ(cpybuf[2], 0x2);

    ASSERT_EQ(bytes_used(), sizeof(initialbuf) - sizeof(cpybuf));
    ASSERT_FALSE(is_full());
    ASSERT_EQ(size_, 240U);
    ASSERT_EQ(bytes_free(), sizeof(cpybuf));
    ASSERT_FALSE(is_empty());
    ASSERT_EQ(head_, buf_.get());
    ASSERT_EQ(tail_, buf_.get() + sizeof(cpybuf));
    ASSERT_FALSE(full_);

    // Now add in some more data; this should go by the tail pointer now,
    // causing the loss of some data
    uint8_t buf2[10];
    for (uint8_t i = 0; i < sizeof(buf2); ++i)
    {
        buf2[i] = i + 10;
    }
    ASSERT_EQ(add_to_memfd(fd, buf2, sizeof(buf2)), static_cast<int>(sizeof(buf2)));

    ASSERT_EQ(read(fd), static_cast<ssize_t>(sizeof(buf2)));

    ASSERT_EQ(bytes_used(), size_);
    ASSERT_TRUE(is_full());
    ASSERT_EQ(size_, 240U);
    ASSERT_EQ(bytes_free(), 0U);
    ASSERT_FALSE(is_empty());
    ASSERT_EQ(head_, buf_.get() + sizeof(buf2));
    ASSERT_EQ(tail_, buf_.get() + sizeof(buf2));
    ASSERT_TRUE(full_);

    // Now look at some data again.  This should be the oldest data available,
    // which in this case is 0xa (we overwrote older data above).
    uint8_t cpybuf2[3]{};
    ASSERT_EQ(memcpy_from(cpybuf2, sizeof(cpybuf2)), static_cast<ssize_t>(sizeof(cpybuf2)));
    ASSERT_EQ(cpybuf2[0], 0xa);
    ASSERT_EQ(cpybuf2[1], 0xb);
    ASSERT_EQ(cpybuf2[2], 0xc);

    ASSERT_EQ(bytes_used(), size_ - sizeof(cpybuf2));
    ASSERT_FALSE(is_full());
    ASSERT_EQ(size_, 240U);
    ASSERT_EQ(bytes_free(), sizeof(cpybuf2));
    ASSERT_FALSE(is_empty());
    ASSERT_EQ(head_, buf_.get() + sizeof(buf2));
    ASSERT_EQ(tail_, buf_.get() + sizeof(buf2) + sizeof(cpybuf2));
    ASSERT_FALSE(full_);

    ::close(fd);
}

TEST_F(RingBufferFixture, memcpy_wrap)
{
    int fd = setup_memfd("ringbuffer_read_test");
    ASSERT_GE(fd, 0);

    // Start by filling up the entire buffer
    uint8_t initialbuf[240];
    for (uint8_t i = 0; i < sizeof(initialbuf); ++i)
    {
        initialbuf[i] = i;
    }
    ASSERT_EQ(add_to_memfd(fd, initialbuf, sizeof(initialbuf)), static_cast<int>(sizeof(initialbuf)));

    ASSERT_EQ(read(fd), static_cast<ssize_t>(sizeof(initialbuf)));

    ASSERT_EQ(bytes_used(), sizeof(initialbuf));
    ASSERT_TRUE(is_full());
    ASSERT_EQ(size_, 240U);
    ASSERT_EQ(bytes_free(), size_ - sizeof(initialbuf));
    ASSERT_FALSE(is_empty());
    ASSERT_EQ(head_, buf_.get());
    ASSERT_EQ(tail_, buf_.get());
    ASSERT_TRUE(full_);

    for (uint8_t i = 0; i < size_; ++i)
    {
        ASSERT_EQ(buf_[i], i);
    }

    // Now copy most of the data out
    uint8_t cpybuf[238]{};
    ASSERT_EQ(memcpy_from(cpybuf, sizeof(cpybuf)), static_cast<ssize_t>(sizeof(cpybuf)));
    for (uint8_t i = 0; i < sizeof(cpybuf); ++i)
    {
        ASSERT_EQ(cpybuf[i], i);
    }

    // Now add in some more data; this should wrap around
    uint8_t buf2[10];
    for (uint8_t i = 0; i < sizeof(buf2); ++i)
    {
        buf2[i] = i + 10;
    }
    ASSERT_EQ(add_to_memfd(fd, buf2, sizeof(buf2)), static_cast<int>(sizeof(buf2)));

    ASSERT_EQ(read(fd), static_cast<ssize_t>(sizeof(buf2)));

    ASSERT_EQ(bytes_used(), 12U);
    ASSERT_FALSE(is_full());
    ASSERT_EQ(size_, 240U);
    ASSERT_EQ(bytes_free(), 228U);
    ASSERT_FALSE(is_empty());
    ASSERT_EQ(head_, buf_.get() + 10);
    ASSERT_EQ(tail_, buf_.get() + 238);
    ASSERT_FALSE(full_);

    // Now look at some data again.  This should be the oldest data available
    // and wraparound to the beginning.
    uint8_t cpybuf2[10]{};
    ASSERT_EQ(memcpy_from(cpybuf2, sizeof(cpybuf2)), static_cast<ssize_t>(sizeof(cpybuf2)));
    ASSERT_EQ(cpybuf2[0], 238);
    ASSERT_EQ(cpybuf2[1], 239);
    ASSERT_EQ(cpybuf2[2], 0xa);
    ASSERT_EQ(cpybuf2[3], 0xb);
    ASSERT_EQ(cpybuf2[4], 0xc);
    ASSERT_EQ(cpybuf2[5], 0xd);
    ASSERT_EQ(cpybuf2[6], 0xe);
    ASSERT_EQ(cpybuf2[7], 0xf);
    ASSERT_EQ(cpybuf2[8], 0x10);
    ASSERT_EQ(cpybuf2[9], 0x11);

    ASSERT_EQ(bytes_used(), 2U);
    ASSERT_FALSE(is_full());
    ASSERT_EQ(size_, 240U);
    ASSERT_EQ(bytes_free(), size_ - 2);
    ASSERT_FALSE(is_empty());
    ASSERT_EQ(head_, buf_.get() + 10);
    ASSERT_EQ(tail_, buf_.get() + 8);
    ASSERT_FALSE(full_);

    ::close(fd);
}

TEST_F(RingBufferFixture, findseq_no_bytes)
{
    uint8_t seq[]{'>', '>', '>'};
    ASSERT_EQ(findseq(seq, sizeof(seq)), -1);
}

TEST_F(RingBufferFixture, findseq_seq_too_large)
{
    uint8_t seq[241]{};
    ASSERT_EQ(findseq(seq, sizeof(seq)), -1);
}

TEST_F(RingBufferFixture, findseq_simple)
{
    int fd = setup_memfd("ringbuffer_read_test");
    ASSERT_GE(fd, 0);

    // Start by adding a known sequence to the buffer.
    uint8_t initialbuf[]{'>', '>', '>'};
    ASSERT_EQ(add_to_memfd(fd, initialbuf, sizeof(initialbuf)), static_cast<int>(sizeof(initialbuf)));

    ASSERT_EQ(read(fd), static_cast<ssize_t>(sizeof(initialbuf)));

    uint8_t seq[]{'>', '>', '>'};
    ASSERT_EQ(findseq(seq, sizeof(seq)), 0);

    ::close(fd);
}

TEST_F(RingBufferFixture, findseq_wrap)
{
    int fd = setup_memfd("ringbuffer_read_test");
    ASSERT_GE(fd, 0);

    // Start by filling most of the buffer.
    uint8_t initialbuf[238]{};
    ASSERT_EQ(add_to_memfd(fd, initialbuf, sizeof(initialbuf)), static_cast<int>(sizeof(initialbuf)));

    ASSERT_EQ(read(fd), static_cast<ssize_t>(sizeof(initialbuf)));

    // Now add in the known sequence, which should span 2 bytes at the end
    // and one byte at the beginning.
    uint8_t buf2[]{'>', '>', '>'};
    ASSERT_EQ(add_to_memfd(fd, buf2, sizeof(buf2)), static_cast<int>(sizeof(buf2)));

    // This first read will return short because we reach the end of the
    // buffer, so call it again to get the rest.
    ASSERT_EQ(read(fd), 2);
    ASSERT_EQ(read(fd), 1);

    uint8_t seq[]{'>', '>', '>'};
    ASSERT_EQ(findseq(seq, sizeof(seq)), 237);

    ::close(fd);
}

TEST_F(RingBufferFixture, findseq_wrap2)
{
    int fd = setup_memfd("ringbuffer_read_test");
    ASSERT_GE(fd, 0);

    // Start by filling most of the buffer.
    uint8_t initialbuf[239]{};
    ASSERT_EQ(add_to_memfd(fd, initialbuf, sizeof(initialbuf)), static_cast<int>(sizeof(initialbuf)));

    ASSERT_EQ(read(fd), static_cast<ssize_t>(sizeof(initialbuf)));

    // Now add in the known sequence, which should span 2 bytes at the end
    // and one byte at the beginning.
    uint8_t buf2[]{'>', '>', '>'};
    ASSERT_EQ(add_to_memfd(fd, buf2, sizeof(buf2)), static_cast<int>(sizeof(buf2)));

    // This first read will return short because we reach the end of the
    // buffer, so call it again to get the rest.
    ASSERT_EQ(read(fd), 1);
    ASSERT_EQ(read(fd), 2);

    uint8_t seq[]{'>', '>', '>'};
    ASSERT_EQ(findseq(seq, sizeof(seq)), 237);

    ::close(fd);
}

TEST_F(RingBufferFixture, findseq_single_byte)
{
    int fd = setup_memfd("ringbuffer_read_test");
    ASSERT_GE(fd, 0);

    // Start by filling most of the buffer.
    uint8_t initialbuf[239];
    for (uint8_t i = 0; i < sizeof(initialbuf); ++i)
    {
        initialbuf[i] = 0x1;
    }
    ASSERT_EQ(add_to_memfd(fd, initialbuf, sizeof(initialbuf)), static_cast<int>(sizeof(initialbuf)));

    ASSERT_EQ(read(fd), static_cast<ssize_t>(sizeof(initialbuf)));

    // Now add in the known sequence, which should span 2 bytes at the end
    // and one byte at the beginning.
    uint8_t buf2[1]{};
    ASSERT_EQ(add_to_memfd(fd, buf2, sizeof(buf2)), static_cast<int>(sizeof(buf2)));

    ASSERT_EQ(read(fd), 1);

    uint8_t seq[]{0x0};
    ASSERT_EQ(findseq(seq, sizeof(seq)), 239);

    ::close(fd);
}

TEST_F(RingBufferFixture, findseq_partial)
{
    int fd = setup_memfd("ringbuffer_read_test");
    ASSERT_GE(fd, 0);

    // Start by filling most of the buffer.
    uint8_t initialbuf[12]{};
    initialbuf[0] = '>';
    initialbuf[1] = '>';
    ASSERT_EQ(add_to_memfd(fd, initialbuf, sizeof(initialbuf)), static_cast<int>(sizeof(initialbuf)));

    ASSERT_EQ(read(fd), static_cast<ssize_t>(sizeof(initialbuf)));

    // Now add in the known sequence, which should span 2 bytes at the end
    // and one byte at the beginning.
    uint8_t buf2[]{'>', '>', '>'};
    ASSERT_EQ(add_to_memfd(fd, buf2, sizeof(buf2)), static_cast<int>(sizeof(buf2)));

    // This first read will return short because we reach the end of the
    // buffer, so call it again to get the rest.
    ASSERT_EQ(read(fd), 3);

    uint8_t seq[]{'>', '>', '>'};
    ASSERT_EQ(findseq(seq, sizeof(seq)), 12);

    ::close(fd);
}

// A test for when the *partial* match is wrapped
TEST_F(RingBufferFixture, findseq_partial_wrap)
{
    int fd = setup_memfd("ringbuffer_read_test");
    ASSERT_GE(fd, 0);

    // Start by filling most of the buffer.
    uint8_t initialbuf[240]{};
    initialbuf[239] = '>';
    ASSERT_EQ(add_to_memfd(fd, initialbuf, sizeof(initialbuf)), static_cast<int>(sizeof(initialbuf)));

    ASSERT_EQ(read(fd), static_cast<ssize_t>(sizeof(initialbuf)));

    // Now add in the rest of the partial sequence, which should wrap to the
    // beginning.
    uint8_t buf2[11]{};
    buf2[0] = '>';
    ASSERT_EQ(add_to_memfd(fd, buf2, sizeof(buf2)), static_cast<int>(sizeof(buf2)));

    // This first read will return short because we reach the end of the
    // buffer, so call it again to get the rest.
    ASSERT_EQ(read(fd), 11);

    // Now add in the real sequence we want to find.
    uint8_t buf3[]{'>', '>', '>'};
    ASSERT_EQ(add_to_memfd(fd, buf3, sizeof(buf3)), static_cast<int>(sizeof(buf3)));

    // This first read will return short because we reach the end of the
    // buffer, so call it again to get the rest.
    ASSERT_EQ(read(fd), 3);

    uint8_t seq[]{'>', '>', '>'};
    ASSERT_EQ(findseq(seq, sizeof(seq)), 237);

    ::close(fd);
}

TEST_F(RingBufferFixture, peek_not_enough_bytes)
{
    uint8_t out[5]{};
    ASSERT_EQ(peek(out, sizeof(out)), -1);
}
