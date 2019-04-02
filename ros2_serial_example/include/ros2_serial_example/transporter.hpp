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

#ifndef ROS2_SERIAL_EXAMPLE__TRANSPORTER_HPP_
#define ROS2_SERIAL_EXAMPLE__TRANSPORTER_HPP_

#include <cstdint>
#include <mutex>
#include <string>

#include "ros2_serial_example/ring_buffer.hpp"

// If you want to allow > 255 topic name/topic types on the serial wire,
// increase the size of this typedef.  Note that it will break on-wire
// serial compatibility.
typedef uint8_t topic_id_size_t;

namespace ros2_to_serial_bridge
{

namespace transport
{

/**
 * The Transporter class provides an abstract class for transporting data over
 * the serial wire.  The Transporter is responsible for marshalling and
 * unmarshalling data in the proper serial wrapper, then calls the base class
 * implementation of node_write() or node_read() to get data to and from the
 * serial wire, respectively.
 *
 * The currently supported serial wire protocols are:
 *
 * PX4 - This is a wire protocol that is 100% compatible with the version from
 * https://github.com/PX4/px4_ros_com.  The benefits to this protocol is that it
 * has a fixed amount of overhead, is easy to understand/implement, and in some
 * cases can be added to a payload without additional copies.  The downsides
 * are that it isn't entirely robust to dropped/corrupt data, and that
 * completely arbitrary data cannot really be sent as a payload (if the payload
 * contains what looks like a header, this could be mistaken for the start of a
 * new packet).
 *
 * COBS - This is an implementation of Consistent Overhead Byte Stuffing,
 * described at https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing.
 * The benefits to this protocol are that it has a small amount of overhead
 * (though it is not fixed), is simple to implement, can send arbitrary data
 * over the wire, and can recover from bit-flip failures fairly easily.  The
 * downsides are that it is a bit harder to understand, and isn't fixed sized
 * overhead.
 */
class Transporter
{
public:
    /**
     * Do Transporter-specific initialization.
     *
     * Since this is an abstract class, it can't be directly constructed, but
     * this constructor is expected to be called during the derived class
     * constructor to setup the Transporter.
     *
     * @param[in] protocol The backend protocol to use; either 'px4' or 'cobs'.
     * @param[in] ring_buffer_size The number of bytes to allocate to the
     *                             underlying ring buffer that is used to
     *                             accept data from the UDP socket.  Larger
     *                             numbers will allow the transport to accept
     *                             larger packets (or more of them), at the
     *                             expense of memory.  It is recommended to
     *                             start with 8192.
     */
    explicit Transporter(const std::string & protocol, size_t ring_buffer_size);
    virtual ~Transporter();

    Transporter(Transporter const &) = delete;
    Transporter& operator=(Transporter const &) = delete;
    Transporter(Transporter &&) = delete;
    Transporter& operator=(Transporter &&) = delete;

    /**
     * Do some transport-specific initialization.
     *
     * This method is provided so that derived classes can do transport-specific
     * initialization.  If the derived class doesn't need to do any of this, it
     * does not need to be overridden.
     *
     * @returns 0 on success, -1 on error.
     */
    virtual int init() {return 0;}

    /**
     * Do some transport-specific close tasks.
     *
     * This method is provided so that derived classes can do transport-specific
     * shutdown.  If the derived class doesn't need to do any of this, it does
     * not need to be overridden.
     *
     * @returns 0 on success, -1 on error.
     */
    virtual int close() {return 0;}

    /**
     * Read some data from the underlying transport and return the payload in
     * out_buffer.
     *
     * This method will call down into the underlying transport and attempt to
     * get some payload bytes.  If there are no valid payloads available, the
     * method will typically return immediately with a return code of 0, though
     * this depends on how the underlying transport is configured.
     *
     * @param[out] topic_ID The topic ID corresponding to the payload (only
     *                      valid if the return value > 0).
     * @param[out] out_buffer The buffer to receive the payload into.
     * @param[in] buffer_len The maximum buffer length to receive the payload into.
     * @returns The payload size on success, 0 if there are no messages
     *          available, and < 0 if the payload couldn't fit into the given
     *          buffer.
     * @throws std::runtime_error If an internal contract was not fulfilled;
     *         this is typically fatal.
     */
    ssize_t read(topic_id_size_t *topic_ID, uint8_t *out_buffer, size_t buffer_len);

    /**
     * Write data from a buffer out to the underlying transport.
     *
     * This method takes payload data passed to it, adds on the serial protocol
     * configured during construction, then calls down to the underlying
     * node_write() method to actually send the data out to the serial port.
     *
     * @param[in] topic_ID The topic ID to add to the message.
     * @param[in] buffer The buffer containing the payload to send.
     * @param[in] length The length of the payload buffer.
     * @returns The payload length written on success, or -1 on error.
     */
    ssize_t write(topic_id_size_t topic_ID, uint8_t const *buffer, size_t data_length);

    // These methods and members are protected because derived classes need
    // access to them.
protected:
    /**
     * Pure virtual method to read data from the underlying transport.
     *
     * Derived classes should override this method to read some data from the
     * underlying transport and store it into the ring buffer.  How much data
     * to read and how long to wait for the data is implementation specific.
     *
     * @returns The number of bytes read, or -1 on error.
     */
    virtual ssize_t node_read() = 0;

    /**
     * Pure virtual method to write data to the underlying transport.
     *
     * Derived classes should override this method to write some data to the
     * underlying transport.  This method should not return until either all
     * of the data has been written or an error occurs.
     *
     * @params[in] buffer The buffer containing the data to write.
     * @params[in] len The number of bytes in the buffer to write.
     * @returns The number of bytes written on success (which must be equal to
     *          len), or -1 on error.
     */
    virtual ssize_t node_write(void *buffer, size_t len) = 0;

    /**
     * Pure virtual method to detect whether the file descriptors are ready.
     *
     * Derived classes should override this method to determine whether the
     * underlying transport is ready to send and receive data.
     *
     * @returns true if the underlying transport is ready to send and receive
     *          data, false otherwise.
     */
    virtual bool fds_OK() = 0;

    /**
     * Method to add the CRC16 of one additional byte.
     *
     * Given an existing CRC16 and a new byte, update the CRC16 to include the
     * data from the new byte.
     *
     * @param[in] crc The existing CRC16.
     * @param[in] data The new byte to add to the CRC16.
     * @returns The new CRC16.
     */
    uint16_t crc16_byte(uint16_t crc, uint8_t data);

    /**
     * Method to calculate the CRC16 of an entire buffer.
     *
     * @param[in] buffer The buffer to calculate the CRC16 over.
     * @param[in] len The length of the buffer to calculate.
     * @returns The CRC16 of the entire buffer.
     */
    uint16_t crc16(uint8_t const *buffer, size_t len);

    impl::RingBuffer ringbuf_;

    // These methods and members are protected because the tests need access
    // to them.
protected:
    enum class SerialProtocol
    {
        PX4,
        COBS,
    };

    /** Get the length of the header.
     *
     * @returns The length of the header.
     */
    size_t get_header_length();

    /**
     * Internal method to find a valid serial message in the ring buffer.
     *
     * This method goes looking through the ring buffer for a valid serial
     * message (what constitues a valid serial message depends on the serial
     * protocol currently in use).  If it finds a valid message, it unpacks it,
     * stuffs the corresponding topic_ID, and fills in the output buffer with
     * the payload.
     *
     * @param[out] topic_ID The topic ID corresponding to the payload (only
     *                      valid if the return value > 0).
     * @param[out] out_buffer The buffer to receive the payload into.
     * @param[in] buffer_len The maximum buffer length to receive the payload into.
     * @returns The payload length on success (which may be 0, and < 0 if a
     *          valid message could not be returned.
     * @throws std::runtime_error If an internal contract was not fulfilled;
     *         this is typically fatal.
     */
    ssize_t find_and_copy_message(topic_id_size_t *topic_ID, uint8_t *out_buffer, size_t buffer_len);

private:
    SerialProtocol backend_protocol_;
    uint8_t seq_{0};
    struct __attribute__((packed)) PX4Header
    {
        uint8_t marker[3];
        topic_id_size_t topic_ID;
        uint8_t seq;
        uint8_t payload_len_h;
        uint8_t payload_len_l;
        uint8_t crc_h;
        uint8_t crc_l;
    };

    struct __attribute__((packed)) COBSHeader
    {
        topic_id_size_t topic_ID;
        uint8_t payload_len_h;
        uint8_t payload_len_l;
        uint8_t crc_h;
        uint8_t crc_l;
    };
    std::mutex write_mutex_;
};

}  // namespace transport
}  // namespace ros2_to_serial_bridge

#endif
