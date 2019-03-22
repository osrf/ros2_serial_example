#include <gtest/gtest.h>

#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

#include <linux/memfd.h>

#include <fcntl.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <unistd.h>

#include "ros2_serial_example/transporter.hpp"

/// HELPERS

static inline int memfd_create(const char *name, unsigned int flags)
{
    return syscall(__NR_memfd_create, name, flags);
}

class TransporterPassThrough : public ros2_to_serial_bridge::transport::Transporter
{
public:
    TransporterPassThrough(const std::string & protocol, size_t ring_buffer_size) : Transporter(protocol, ring_buffer_size)
    {
    }

    ~TransporterPassThrough() override
    {
    }

    ssize_t node_read() override
    {
        return 0;
    }

    ssize_t node_write(void *buffer, size_t len) override
    {
        (void)buffer;
        (void)len;
        return 0;
    }

    bool fds_OK() override
    {
        return true;
    }
};

class TransporterFixture : public ros2_to_serial_bridge::transport::Transporter, public testing::Test
{
public:
    TransporterFixture(const std::string & protocol) : Transporter(protocol, 240)
    {
        // Setup the memory fd
        memfd_ = memfd_create(protocol.c_str(), MFD_CLOEXEC);
        if (memfd_ < 0)
        {
            throw std::runtime_error("Failed to create memfd");
        }

        // The RingBuffer class essentially assumes that this file descriptor
        // is non-blocking
        if (::fcntl(memfd_, F_SETFL, O_NONBLOCK) != 0)
        {
            ::close(memfd_);
            throw std::runtime_error("Failed to set memfd nonblocking");
        }
    }

    virtual ~TransporterFixture()
    {
        ::close(memfd_);
    }

    // Add some additional data to the memfd.  After this call the file pointer
    // will be at the start of the new memory.
    int add_to_memfd(uint8_t *buf, size_t bufsize)
    {
        off_t offset = ::lseek(memfd_, 0, SEEK_CUR);
        if (offset < 0)
        {
            return -1;
        }

        if (::ftruncate(memfd_, offset + bufsize) != 0)
        {
            return -1;
        }

        if (::lseek(memfd_, offset, SEEK_SET) != offset)
        {
            return -1;
        }

        if (::write(memfd_, buf, bufsize) != static_cast<int>(bufsize))
        {
            return -1;
        }

        if (::lseek(memfd_, offset, SEEK_SET) != offset)
        {
            return -1;
        }

        return bufsize;
    }

    ssize_t node_read() override
    {
        return ringbuf_.read(memfd_);
    }

    ssize_t node_write(void *buffer, size_t len) override
    {
        written_data_ = std::unique_ptr<uint8_t[]>(new uint8_t[len]);
        ::memcpy(written_data_.get(), buffer, len);
        return len;
    }

    bool fds_OK() override
    {
        return test_fds_ok_;
    }

protected:
    // This variable is used to hang on to data written by tests so it can be
    // examined.
    std::unique_ptr<uint8_t[]> written_data_;

    // This file descriptor connects to a memory fd which tests can fill with
    // data of their choosing.  That data will be returned when a test
    // calls read()
    int memfd_;

    // This variable is used by the tests to control whether fds_OK returns
    // true or false (true by default)
    bool test_fds_ok_{true};
};

// This fixture allows us access to the protected methods of Transporter
class PX4TransporterFixture : public TransporterFixture
{
public:
    PX4TransporterFixture() : TransporterFixture("px4")
    {
    }
};

// This fixture allows us access to the protected methods of Transporter
class COBSTransporterFixture : public TransporterFixture
{
public:
    COBSTransporterFixture() : TransporterFixture("cobs")
    {
    }
};

std::vector<uint8_t> setup_px4_test_data()
{
    // The amount of data we need is 3 bytes for the marker, plus the size of
    // the topic id, plus 5 bytes for the sequence, payload, and CRC, plus 4
    // bytes for the payload.
    std::vector<uint8_t> test_data(3 + sizeof(topic_id_size_t) + 5 + 4);

    topic_id_size_t topic_ID = 0xa;
    size_t i = 0;
    test_data[i++] = '>';  // marker 1
    test_data[i++] = '>';  // marker 2
    test_data[i++] = '>';  // marker 3
    ::memcpy(&test_data[i], &topic_ID, sizeof(topic_id_size_t));  // topic id
    i += sizeof(topic_id_size_t);
    test_data[i++] = 0x00;  // sequence number
    test_data[i++] = 0x00;  // payload length high
    test_data[i++] = 0x04;  // payload length low
    test_data[i++] = 0x6d;  // crc high
    test_data[i++] = 0x10;  // crc low
    test_data[i++] = 0x05;  // payload 1
    test_data[i++] = 0x01;  // payload 2
    test_data[i++] = 0x02;  // payload 3
    test_data[i++] = 0x03;  // payload 4

    return test_data;
}

std::vector<uint8_t> setup_cobs_test_data()
{
    // // The amount of data we need is 4 bytes for the header, plus the size of
    // // the topic id, plus one byte for the final 0, plus 4 bytes for the payload.
    // std::vector<uint8_t> test_data(4 + sizeof(topic_id_size_t) + 1 + 4);
    // topic_id_size_t topic_ID = 0xa;
    // size_t i = 0;

    if (sizeof(topic_id_size_t) == 1)
    {
        return std::vector<uint8_t>{
            0x2, 0xa, 0x8, 0x4, 0x6d, 0x10, 0x5, 0x1, 0x2, 0x3, 0x0,
        };
    }
    else if (sizeof(topic_id_size_t) == 2)
    {
        return std::vector<uint8_t>{
            0x2, 0xa, 0x1, 0x8, 0x4, 0x6d, 0x10, 0x5, 0x1, 0x2, 0x3, 0x0,
        };
    }

    throw std::runtime_error("Invalid topic ID size");
}

TEST(TransporterPassThrough, px4_protocol)
{
    TransporterPassThrough trans("px4", 1024);

    ASSERT_EQ(trans.init(), 0);

    ASSERT_EQ(trans.close(), 0);
}

TEST(TransporterPassThrough, cobs_protocol)
{
    TransporterPassThrough trans("cobs", 1024);

    ASSERT_EQ(trans.init(), 0);

    ASSERT_EQ(trans.close(), 0);
}

TEST(TransporterPassThrough, invalid_protocol)
{
    try
    {
        TransporterPassThrough trans("foo", 1024);
    }
    catch (const std::runtime_error & e)
    {
        if (std::string(e.what()).find("Invalid protocol") == std::string::npos)
        {
          FAIL() << "Expected error msg containing: Invalid protocol" << std::endl
                 << "Saw error msg: " << std::endl
                 << e.what() << std::endl;
        }
    }
    catch (const std::exception & e)
    {
      FAIL() << "Expected exception of type " "std::runtime_error" << std::endl
             << "Saw exception of type: " << typeid(e).name() << std::endl;
    }
}

TEST_F(PX4TransporterFixture, get_header_length)
{
    ASSERT_EQ(get_header_length(), sizeof(topic_id_size_t) + 8U);
}

TEST_F(PX4TransporterFixture, crc16_byte)
{
    ASSERT_EQ(crc16_byte(0, 0), 0);
    ASSERT_EQ(crc16_byte(0, 1), 0xc0c1);
}

TEST_F(PX4TransporterFixture, crc16)
{
    std::unique_ptr<uint8_t[]> buf = std::unique_ptr<uint8_t[]>(new uint8_t[4]{});

    ASSERT_EQ(crc16(buf.get(), 4), 0);
}

TEST_F(PX4TransporterFixture, write_fds_not_ok)
{
    std::unique_ptr<uint8_t[]> buf = std::unique_ptr<uint8_t[]>(new uint8_t[4]{});

    test_fds_ok_ = false;
    ASSERT_EQ(write(0, buf.get(), 4), -1);
}

TEST_F(PX4TransporterFixture, write_nullptr)
{
    ASSERT_EQ(write(0, nullptr, 0), 0);
}

TEST_F(PX4TransporterFixture, write_zero_data)
{
    std::unique_ptr<uint8_t[]> buf = std::unique_ptr<uint8_t[]>(new uint8_t[4]{});
    ASSERT_EQ(write(0, buf.get(), 0), -1);
}

TEST_F(PX4TransporterFixture, write_nullptr_with_length)
{
    ASSERT_EQ(write(0, nullptr, 4), -1);
}

TEST_F(PX4TransporterFixture, write)
{
    std::unique_ptr<uint8_t[]> buf = std::unique_ptr<uint8_t[]>(new uint8_t[4]{});
    buf.get()[0] = 0x5;
    buf.get()[1] = 0x1;
    buf.get()[2] = 0x2;
    buf.get()[3] = 0x3;

    ASSERT_EQ(write(0xa, buf.get(), 4), 4);

    std::vector<uint8_t> expected = setup_px4_test_data();

    for (size_t i = 0; i < expected.size(); ++i)
    {
        ASSERT_EQ(written_data_.get()[i], expected[i]);
    }
}

TEST_F(PX4TransporterFixture, read_null_topic_id)
{
    std::unique_ptr<uint8_t[]> buf = std::unique_ptr<uint8_t[]>(new uint8_t[4]{});
    ASSERT_EQ(read(nullptr, buf.get(), 4), -1);
}

TEST_F(PX4TransporterFixture, read_null_buffer)
{
    topic_id_size_t topic_id;
    ASSERT_EQ(read(&topic_id, nullptr, 4), -1);
}

TEST_F(PX4TransporterFixture, read_fds_not_ok)
{
    std::unique_ptr<uint8_t[]> buf = std::unique_ptr<uint8_t[]>(new uint8_t[4]{});
    topic_id_size_t topic_id;

    test_fds_ok_ = false;
    ASSERT_EQ(read(&topic_id, buf.get(), 4), -1);
}

TEST_F(PX4TransporterFixture, read_message)
{
    std::unique_ptr<uint8_t[]> buf = std::unique_ptr<uint8_t[]>(new uint8_t[4]{});

    std::vector<uint8_t> read_data = setup_px4_test_data();

    add_to_memfd(&read_data[0], read_data.size());

    topic_id_size_t topic_id;
    ASSERT_EQ(read(&topic_id, buf.get(), 4), 4);
    ASSERT_EQ(topic_id, 0xa);
    ASSERT_EQ(buf.get()[0], 0x05);
    ASSERT_EQ(buf.get()[1], 0x01);
    ASSERT_EQ(buf.get()[2], 0x02);
    ASSERT_EQ(buf.get()[3], 0x03);
}

TEST_F(PX4TransporterFixture, read_message_already_available)
{
    std::unique_ptr<uint8_t[]> buf = std::unique_ptr<uint8_t[]>(new uint8_t[4]{});

    std::vector<uint8_t> read_data = setup_px4_test_data();

    add_to_memfd(&read_data[0], read_data.size());
    ASSERT_EQ(node_read(), static_cast<ssize_t>(read_data.size()));

    topic_id_size_t topic_id;
    ASSERT_EQ(read(&topic_id, buf.get(), 4), 4);
    ASSERT_EQ(topic_id, 0xa);
    ASSERT_EQ(buf.get()[0], 0x05);
    ASSERT_EQ(buf.get()[1], 0x01);
    ASSERT_EQ(buf.get()[2], 0x02);
    ASSERT_EQ(buf.get()[3], 0x03);
}

TEST_F(COBSTransporterFixture, get_header_length)
{
    ASSERT_EQ(get_header_length(), sizeof(topic_id_size_t) + 4U);
}

TEST_F(COBSTransporterFixture, write)
{
    std::unique_ptr<uint8_t[]> buf = std::unique_ptr<uint8_t[]>(new uint8_t[4]{});
    buf.get()[0] = 0x5;
    buf.get()[1] = 0x1;
    buf.get()[2] = 0x2;
    buf.get()[3] = 0x3;

    ASSERT_EQ(write(0xa, buf.get(), 4), 4);

    std::vector<uint8_t> expected = setup_cobs_test_data();

    for (size_t i = 0; i < expected.size(); ++i)
    {
        ASSERT_EQ(written_data_.get()[i], expected[i]);
    }
}

std::vector<uint8_t> setup_cobs_long_data()
{
    if (sizeof(topic_id_size_t) == 1)
    {
        std::vector<uint8_t> test_data{
            0xff, 0xa, 0x1, 0x2c, 0xb6, 0x4a
        };

        size_t header_size = test_data.size();

        test_data.resize(header_size + 300 + 1 + 1);
        test_data[test_data.size() - 1] = 0x0;

        for (size_t i = header_size; i < test_data.size() - 1; ++i)
        {
            test_data[i] = 0x1;
        }

        test_data[255] = 0x34;

        return test_data;
    }
    else if (sizeof(topic_id_size_t) == 2)
    {
        std::vector<uint8_t> test_data{
            0x2, 0xa, 0xff, 0x1, 0x2c, 0xb6, 0x4a
        };

        size_t header_size = test_data.size();

        test_data.resize(header_size + 300 + 1 + 1);
        test_data[test_data.size() - 1] = 0x0;

        for (size_t i = header_size; i < test_data.size() - 1; ++i)
        {
            test_data[i] = 0x1;
        }

        test_data[257] = 0x33;

        return test_data;
    }

    throw std::runtime_error("Unsupported topic_id_size_t size");
}

TEST_F(COBSTransporterFixture, write_long_sequence)
{
    std::unique_ptr<uint8_t[]> buf = std::unique_ptr<uint8_t[]>(new uint8_t[300]{});
    for (size_t i = 0; i < 300; ++i)
    {
        buf[i] = 0x1;
    }

    ASSERT_EQ(write(0xa, buf.get(), 300), 300);

    std::vector<uint8_t> expected = setup_cobs_long_data();

    for (size_t i = 0; i < expected.size(); ++i)
    {
        ASSERT_EQ(written_data_.get()[i], expected[i]);
    }
}

TEST_F(COBSTransporterFixture, read_message)
{
    std::unique_ptr<uint8_t[]> buf = std::unique_ptr<uint8_t[]>(new uint8_t[4]{});

    std::vector<uint8_t> read_data = setup_cobs_test_data();
    add_to_memfd(&read_data[0], read_data.size());

    topic_id_size_t topic_id;
    ASSERT_EQ(read(&topic_id, buf.get(), 4), 4);
    ASSERT_EQ(topic_id, 0xa);
    ASSERT_EQ(buf.get()[0], 0x05);
    ASSERT_EQ(buf.get()[1], 0x01);
    ASSERT_EQ(buf.get()[2], 0x02);
    ASSERT_EQ(buf.get()[3], 0x03);
}

TEST_F(COBSTransporterFixture, read_message_already_available)
{
    std::unique_ptr<uint8_t[]> buf = std::unique_ptr<uint8_t[]>(new uint8_t[4]{});

    std::vector<uint8_t> read_data = setup_cobs_test_data();
    add_to_memfd(&read_data[0], read_data.size());

    ASSERT_EQ(node_read(), static_cast<ssize_t>(read_data.size()));

    topic_id_size_t topic_id;
    ASSERT_EQ(read(&topic_id, buf.get(), 4), 4);
    ASSERT_EQ(topic_id, 0xa);
    ASSERT_EQ(buf.get()[0], 0x05);
    ASSERT_EQ(buf.get()[1], 0x01);
    ASSERT_EQ(buf.get()[2], 0x02);
    ASSERT_EQ(buf.get()[3], 0x03);
}
