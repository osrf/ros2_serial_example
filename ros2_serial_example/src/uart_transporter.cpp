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
// but modified to split UART code out of the original transporter.

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include "ros2_serial_example/transporter.hpp"
#include "ros2_serial_example/uart_transporter.hpp"

namespace ros2_to_serial_bridge
{

namespace transport
{

uint32_t baud_number_to_rate(uint32_t baud)
{
    // This is a table of the standard baudrates as defined in
    // /usr/include/asm-generic/termbits.h
    static std::map<uint32_t, uint32_t> BaudNumberToRate{
        {0,       B0},
        {50,      B50},
        {75,      B75},
        {110,     B110},
        {134,     B134},
        {150,     B150},
        {200,     B200},
        {300,     B300},
        {600,     B600},
        {1200,    B1200},
        {1800,    B1800},
        {2400,    B2400},
        {4800,    B4800},
        {9600,    B9600},
        {19200,   B19200},
        {38400,   B38400},
        {57600,   B57600},
        {115200,  B115200},
        {230400,  B230400},
        {460800,  B460800},
        {500000,  B500000},
        {576000,  B576000},
        {921600,  B921600},
        {1000000, B1000000},
        {1500000, B1500000},
        {2000000, B2000000},
        {2500000, B2500000},
        {3000000, B3000000},
        {3500000, B3500000},
        {4000000, B4000000},
    };

    if (BaudNumberToRate.count(baud) == 0)
    {
        throw std::runtime_error("Invalid baudrate");
    }
    return BaudNumberToRate[baud];
}

UARTTransporter::UARTTransporter(const std::string & uart_name,
                                 const std::string & protocol,
                                 uint32_t baudrate,
                                 uint32_t read_poll_ms,
                                 size_t ring_buffer_size):
    Transporter(protocol, ring_buffer_size),
    uart_name_(uart_name),
    read_poll_ms_(read_poll_ms)
{
    baudrate_ = baud_number_to_rate(baudrate);
}

UARTTransporter::~UARTTransporter()
{
    close();
}

int UARTTransporter::init()
{
    if (fds_OK())
    {
        ::fprintf(stderr, "Cannot re-init; call close first\n");
        return -1;
    }

    // Open a serial port
    uart_fd_ = ::open(uart_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);

    if (uart_fd_ < 0)
    {
        ::fprintf(stderr, "failed to open device: %s (%d)\n", uart_name_.c_str(), errno);
        return -errno;
    }

    // If a baudrate > 0 is specified, we setup the UART with that rate.
    if (baudrate_ != B0)
    {
        // Try to set baud rate
        struct termios uart_config{};
        int termios_state;

        // Back up the original uart configuration to restore it after exit
        if ((termios_state = ::tcgetattr(uart_fd_, &uart_config)) < 0)
        {
            int errno_bkp = errno;
            ::fprintf(stderr, "ERR GET CONF %s: %d (%d)\n", uart_name_.c_str(), termios_state, errno);
            close();
            return -errno_bkp;
        }

        // Set up the UART for non-canonical binary communication: 8 bits, 1 stop bit, no parity,
        // no flow control, no modem control
        uart_config.c_iflag &= ~(INPCK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXANY | IXOFF);
        uart_config.c_iflag |= IGNBRK | IGNPAR;

        uart_config.c_oflag &= ~(OPOST | ONLCR | OCRNL | ONOCR | ONLRET | OFILL | NLDLY | VTDLY);
        uart_config.c_oflag |= NL0 | VT0;

        uart_config.c_cflag &= ~(CSIZE | CSTOPB | PARENB);
        uart_config.c_cflag |= CS8 | CREAD | CLOCAL;

        uart_config.c_lflag &= ~(ISIG | ICANON | ECHO | TOSTOP | IEXTEN);

        // Set baud rate
        if (::cfsetispeed(&uart_config, baudrate_) < 0 || ::cfsetospeed(&uart_config, baudrate_) < 0)
        {
            int errno_bkp = errno;
            ::fprintf(stderr, "ERR SET BAUD %s: %d (%d)\n", uart_name_.c_str(), termios_state, errno);
            close();
            return -errno_bkp;
        }

        if ((termios_state = ::tcsetattr(uart_fd_, TCSANOW, &uart_config)) < 0)
        {
            int errno_bkp = errno;
            ::fprintf(stderr, "ERR SET CONF %s (%d)\n", uart_name_.c_str(), errno);
            close();
            return -errno_bkp;
        }
    }

    // Flush out any pending data in the file descriptor.
    uint8_t aux[64];
    while (0 < ::read(uart_fd_, &aux, 64))
    {
        ::usleep(1000);
    }

    poll_fd_[0].fd = uart_fd_;
    poll_fd_[0].events = POLLIN;

    return 0;
}

bool UARTTransporter::fds_OK()
{
    return (-1 != uart_fd_);
}

int UARTTransporter::close()
{
    if (-1 != uart_fd_)
    {
        ::close(uart_fd_);
        uart_fd_ = -1;
        ::memset(&poll_fd_, 0, sizeof(poll_fd_));
    }

    return 0;
}

ssize_t UARTTransporter::node_read()
{
    if (!fds_OK())
    {
        return -1;
    }

    ssize_t ret = 0;
    int r = ::poll(reinterpret_cast<struct pollfd *>(poll_fd_), 1, read_poll_ms_);

    if (r == 1 && (poll_fd_[0].revents & POLLIN) != 0)
    {
        ret = ringbuf_.read(uart_fd_);
    }

    return ret;
}

ssize_t UARTTransporter::node_write(void *buffer, size_t len)
{
    if (nullptr == buffer || !fds_OK())
    {
        return -1;
    }

    uint32_t intr_times = 0;

    // Ensure that the entire buffer gets out to the file descriptor (unless a
    // fatal error occurs)
    uint8_t *b = static_cast<uint8_t *>(buffer);
    size_t n = len;
    while (n > 0)
    {
        ssize_t ret = ::write(uart_fd_, b, n);
        if (ret == -1)
        {
            if (errno == EINTR || errno == EAGAIN)
            {
                // We can get an EINTR/EAGAIN for a temporary failure (for
                // instance, the remote end of a pipe is temporarily busy and not
                // draining the data fast enough), or for a more permanent one
                // (for instance, nothing is draining the remote end of a pipe).
                // Just doing "continue" here in the latter case means that we'll
                // be stuck in this loop forever, burning CPU.  Thus, we add a
                // small delay and a counter in here.  If we get into here
                // `write_timeout_us` times in a row (with no data being
                // transferred in between), we presume this is a permanent failure
                // and return an error to the application.
                intr_times++;
                if (intr_times > write_timeout_us_)
                {
                    // Too many failures, set an errno and get out.
                    errno = EBUSY;
                    break;
                }
                ::usleep(1);
                continue;
            }

            break;
        }
        intr_times = 0;
        n -= ret;
        b += ret;
    }

    return (n > 0) ? -1 : len;
}

}  // namespace transport
}  // namespace ros2_to_serial_bridge
