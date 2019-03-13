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

#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include <unistd.h>

#include <fastcdr/Cdr.h>
#include <fastcdr/FastCdr.h>

#include "ros2_serial_example/transporter.hpp"
#include "ros2_serial_example/uart_transporter.hpp"

constexpr int BUFFER_SIZE = 1024;

static void usage(const char *name)
{
    ::printf("Usage: %s [options]\n\n"
             "  -d <device> UART device. Default /dev/ttyACM0\n"
             "  -h          Print this help message\n",
             name);
}

volatile sig_atomic_t running = 1;

static void signal_handler(int signum)
{
    (void)signum;
    running = 0;
}

void read_thread_func(ros2_to_serial_bridge::transport::Transporter * transporter)
{
    // We use a unique_ptr here both to make this a heap allocation and to quiet
    // non-owning pointer warnings from clang-tidy
    std::unique_ptr<uint8_t[]> data_buffer(new uint8_t[BUFFER_SIZE]);
    ssize_t length = 0;
    topic_id_size_t topic_ID;

    while (running != 0)
    {
        // Process data coming over serial
        if ((length = transporter->read(&topic_ID, data_buffer.get(), BUFFER_SIZE)) > 0)
        {
            ::fprintf(stderr, "Topic ID: %d, data: ", topic_ID);
            for (ssize_t i = 0; i < length; ++i)
            {
              ::fprintf(stderr, "0x%x ", *(data_buffer.get() + i));
            }
            ::fprintf(stderr, "\n");
        }
    }
}

int main(int argc, char *argv[])
{
    std::string device{};
    uint32_t baudrate = 0;
    std::string serial_protocol{"px4"};

    int ch;
    while ((ch = ::getopt(argc, argv, "b:d:hs:")) != EOF)
    {
        switch (ch)
        {
        case 'b':
            if (optarg != nullptr)
            {
                char *endptr;
                errno = 0;
                baudrate = ::strtoul(optarg, &endptr, 10);
                if (errno == ERANGE)
                {
                    ::fprintf(stderr, "Invalid baudrate (outside of valid range)\n");
                    return 1;
                }
                if (*optarg == '\0' || *endptr != '\0')
                {
                    ::fprintf(stderr, "Entire baudrate not converted; must be a number\n");
                    return 1;
                }
            }
            break;
        case 'd':
            if (optarg != nullptr)
            {
                device = optarg;
            }
            break;
        case 'h':
            usage(argv[0]);
            return 0;
        case 's':
            if (optarg != nullptr)
            {
                serial_protocol = optarg;
            }
            break;
        default:
            usage(argv[0]);
            return 1;
        }
    }

    if (optind < argc)
    {
        usage(argv[0]);
        return 1;
    }

    std::unique_ptr<ros2_to_serial_bridge::transport::Transporter> transporter = std::make_unique<ros2_to_serial_bridge::transport::UARTTransporter>(device, serial_protocol, baudrate, 100, 8192);

    if (transporter->init() < 0)
    {
        return 1;
    }

    ::signal(SIGINT, signal_handler);

    std::thread read_thread(read_thread_func, transporter.get());

    size_t headlen = transporter->get_header_length();

    // We use a unique_ptr here both to make this a heap allocation and to quiet
    // non-owning pointer warnings from clang-tidy
    std::unique_ptr<uint8_t[]> data_buffer(new uint8_t[BUFFER_SIZE]);
    std::unique_ptr<uint8_t[]> data_buffer2(new uint8_t[BUFFER_SIZE]);

    while (running != 0)
    {
        // With the current configuration, topic 9 is a std_msgs/String topic.
        eprosima::fastcdr::FastBuffer cdrbuffer(reinterpret_cast<char *>(data_buffer.get()) + headlen, BUFFER_SIZE - headlen);
        eprosima::fastcdr::Cdr scdr(cdrbuffer);
        scdr << "aa";
        if (transporter->write(9, data_buffer.get(), scdr.getSerializedDataLength()) < 0)
        {
            ::fprintf(stderr, "Failed to write topic %d: %s\n", 9, ::strerror(errno));
        }

        // With the current configuration, topic 12 is a std_msgs/UInt16 topic.
        eprosima::fastcdr::FastBuffer cdrbuffer2(reinterpret_cast<char *>(data_buffer2.get()) + headlen, BUFFER_SIZE - headlen);
        eprosima::fastcdr::Cdr scdr2(cdrbuffer2);
        scdr2 << 256;
        if (transporter->write(12, data_buffer2.get(), scdr2.getSerializedDataLength()) < 0)
        {
            ::fprintf(stderr, "Failed to write topic %d: %s\n", 12, ::strerror(errno));
        }

        // every 100 milliseconds
        ::usleep(100000);
    }

    read_thread.join();

    transporter->close();

    return 0;
}
