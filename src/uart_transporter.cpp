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
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <sys/socket.h>

#include "ros2_serial_example/transporter.hpp"
#include "ros2_serial_example/uart_transporter.hpp"

UART_node::UART_node(const char *_uart_name, uint32_t _baudrate, uint32_t _poll_ms):
    uart_fd(-1),
    baudrate(_baudrate),
    poll_ms(_poll_ms)

{
    if (nullptr != _uart_name)
    {
        ::strcpy(uart_name, _uart_name);
    }
}

UART_node::~UART_node()
{
    close();
}

int UART_node::init()
{
    // Open a serial port
    uart_fd = ::open(uart_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (uart_fd < 0)
    {
        ::printf("failed to open device: %s (%d)\n", uart_name, errno);
        return -errno;
    }

    // If using shared UART, no need to set it up
    if (baudrate == 0)
    {
        return uart_fd;
    }

    // Try to set baud rate
    struct termios uart_config;
    int termios_state;

    // Back up the original uart configuration to restore it after exit
    if ((termios_state = tcgetattr(uart_fd, &uart_config)) < 0)
    {
        int errno_bkp = errno;
        ::printf("ERR GET CONF %s: %d (%d)\n", uart_name, termios_state, errno);
        close();
        return -errno_bkp;
    }

    //Set up the UART for non-canonical binary communication: 8 bits, 1 stop bit, no parity,
    //no flow control, no modem control
    uart_config.c_iflag &= !(INPCK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXANY | IXOFF);
    uart_config.c_iflag |= IGNBRK | IGNPAR;

    uart_config.c_oflag &= !(OPOST | ONLCR | OCRNL | ONOCR | ONLRET | OFILL | NLDLY | VTDLY);
    uart_config.c_oflag |= NL0 | VT0;

    uart_config.c_cflag &= !(CSIZE | CSTOPB | PARENB);
    uart_config.c_cflag |= CS8 | CREAD | CLOCAL;

    uart_config.c_lflag &= !(ISIG | ICANON | ECHO | TOSTOP | IEXTEN);

    // USB serial is indicated by /dev/ttyACM0
    // TODO(clalancette): this is arbitrary, probably remove this
    if (::strcmp(uart_name, "/dev/ttyACM0") != 0 && ::strcmp(uart_name, "/dev/ttyACM1") != 0)
    {
        // Set baud rate
        if (::cfsetispeed(&uart_config, baudrate) < 0 || ::cfsetospeed(&uart_config, baudrate) < 0)
        {
            int errno_bkp = errno;
            ::printf("ERR SET BAUD %s: %d (%d)\n", uart_name, termios_state, errno);
            close();
            return -errno_bkp;
        }
    }

    if ((termios_state = ::tcsetattr(uart_fd, TCSANOW, &uart_config)) < 0)
    {
        int errno_bkp = errno;
        ::printf("ERR SET CONF %s (%d)\n", uart_name, errno);
        close();
        return -errno_bkp;
    }

    uint8_t aux[64];
    bool flush = false;

    while (0 < ::read(uart_fd, (void *)&aux, 64))
    {
        //printf("%s ", aux);
        flush = true;
        usleep(1000);
    }

    if (flush)
    {
        ::printf("flush\n");
    }
    else
    {
        ::printf("no flush\n");
    }

    poll_fd[0].fd = uart_fd;
    poll_fd[0].events = POLLIN;

    return uart_fd;
}

bool UART_node::fds_OK()
{
    return (-1 != uart_fd);
}

uint8_t UART_node::close()
{
    if (-1 != uart_fd)
    {
        ::printf("Close UART\n");
        ::close(uart_fd);
        uart_fd = -1;
        ::memset(&poll_fd, 0, sizeof(poll_fd));
    }

    return 0;
}

ssize_t UART_node::node_read()
{
    if (!fds_OK())
    {
        return -1;
    }

    ssize_t ret = 0;
    int r = ::poll(poll_fd, 1, poll_ms);

    if (r == 1 && (poll_fd[0].revents & POLLIN))
    {
        ret = ringbuf.read(uart_fd);
    }

    return ret;
}

ssize_t UART_node::node_write(void *buffer, size_t len)
{
    if (nullptr == buffer || !fds_OK())
    {
        return -1;
    }

    return ::write(uart_fd, buffer, len);
}
