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
// but modified to split UDP code out of the original transporter.

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <unistd.h>

#include "ros2_serial_example/transporter.hpp"
#include "ros2_serial_example/udp_transporter.hpp"

namespace ros2_to_serial_bridge
{

namespace transport
{

UDPTransporter::UDPTransporter(const std::string & protocol,
                               uint16_t recv_port,
                               uint16_t send_port,
                               uint32_t read_poll_ms,
                               size_t ring_buffer_size):
    Transporter(protocol, ring_buffer_size),
    recv_port_(recv_port),
    send_port_(send_port),
    read_poll_ms_(read_poll_ms)
{
    if (recv_port_ == 0 || send_port_ == 0)
    {
        throw std::runtime_error("Invalid send or receive port, must be between 1 and 65535 inclusive");
    }
}

UDPTransporter::~UDPTransporter()
{
    close();
}

int UDPTransporter::init()
{
    if (fds_OK())
    {
        ::fprintf(stderr, "Cannot re-init; call close first\n");
        return -1;
    }

    // Setup the receiver
    struct sockaddr_in receiver_inaddr{};

    // udp socket data
    receiver_inaddr.sin_family = AF_INET;
    receiver_inaddr.sin_port = ::htons(recv_port_);
    receiver_inaddr.sin_addr.s_addr = ::htonl(INADDR_ANY);

    if ((recv_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        ::fprintf(stderr, "create socket failed: %s\n", ::strerror(errno));
        return -errno;
    }

    if (::bind(recv_fd_, reinterpret_cast<struct sockaddr *>(&receiver_inaddr),
               sizeof(receiver_inaddr)) < 0)
    {
        ::fprintf(stderr, "bind failed %s\n", ::strerror(errno));
        return -1;
    }

    // Now make the incoming socket non-blocking.
    int flags = ::fcntl(recv_fd_, F_GETFL, 0);
    if (flags < 0)
    {
        ::fprintf(stderr, "Failed to get flags for UDP receive: %s\n",
                  ::strerror(errno));
        return -1;
    }
    flags |= O_NONBLOCK;
    if (::fcntl(recv_fd_, F_SETFL, flags) < 0)
    {
        ::fprintf(stderr, "Failed to set flags for UDP receive: %s\n",
                  ::strerror(errno));
        return -1;
    }

    poll_fd_[0].fd = recv_fd_;
    poll_fd_[0].events = POLLIN;

    // Setup the sender
    if ((send_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        ::fprintf(stderr, "create socket failed: %s\n", ::strerror(errno));
        return -1;
    }

    send_outaddr.sin_family = AF_INET;
    send_outaddr.sin_port = ::htons(send_port_);

    if (::inet_aton("127.0.0.1", &send_outaddr.sin_addr) == 0)
    {
        ::fprintf(stderr, "inet_aton() failed: %s\n", ::strerror(errno));
        return -1;
    }

    return 0;
}

bool UDPTransporter::fds_OK()
{
    return (-1 != recv_fd_ && -1 != send_fd_);
}

int UDPTransporter::close()
{
    if (-1 != recv_fd_)
    {
        ::shutdown(recv_fd_, SHUT_RDWR);
        ::close(recv_fd_);
        recv_fd_ = -1;
    }

    if (-1 != send_fd_)
    {
        ::shutdown(send_fd_, SHUT_RDWR);
        ::close(send_fd_);
        send_fd_ = -1;
    }

    ::memset(&poll_fd_, 0, sizeof(poll_fd_));

    return 0;
}

ssize_t UDPTransporter::node_read()
{
    if (!fds_OK())
    {
        return -1;
    }

    ssize_t ret = 0;
    int r = ::poll(reinterpret_cast<struct pollfd *>(poll_fd_), 1, read_poll_ms_);

    if (r == 1 && (poll_fd_[0].revents & POLLIN) != 0)
    {
        ret = ringbuf_.read(recv_fd_);
    }

    return ret;
}

ssize_t UDPTransporter::node_write(void *buffer, size_t len)
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
        ssize_t ret = ::sendto(send_fd_, b, n, 0, reinterpret_cast<struct sockaddr *>(&send_outaddr), sizeof(send_outaddr));
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
