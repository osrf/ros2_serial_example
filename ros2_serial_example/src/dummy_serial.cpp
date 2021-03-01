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

#include "ros2_serial_msgs/msg/serial_mapping.hpp"
#include "ros2_serial_msgs/msg/detail/serial_mapping__rosidl_typesupport_fastrtps_cpp.hpp"

#include "ros2_serial_example/transporter.hpp"
#include "ros2_serial_example/uart_transporter.hpp"

constexpr int BUFFER_SIZE = 1024;

static void usage(const char *name)
{
    ::printf("Usage: %s [options]\n\n"
             "  -b <baudrate> Baudrate to use for the device\n"
             "  -d <device>   UART device; must be specified\n"
             "  -h            Print this help message\n"
             "  -s <protocol> Serial protocol to use; currently supported are\n"
             "                'cobs' (default) and 'px4'\n",
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
        if ((length = transporter->read(&topic_ID, data_buffer.get(), BUFFER_SIZE)) >= 0)
        {
            if (topic_ID == 0)
            {
                // The other side is requesting a manifest
                ros2_serial_msgs::msg::SerialMapping serial_mapping;

                serial_mapping.topic_names.push_back("chatter");
                serial_mapping.serial_mappings.push_back(9);
                serial_mapping.types.push_back("std_msgs/String");
                serial_mapping.direction.push_back(ros2_serial_msgs::msg::SerialMapping::SERIALTOROS2);

                serial_mapping.topic_names.push_back("uint16topic");
                serial_mapping.serial_mappings.push_back(12);
                serial_mapping.types.push_back("std_msgs/UInt16");
                serial_mapping.direction.push_back(ros2_serial_msgs::msg::SerialMapping::SERIALTOROS2);

                serial_mapping.topic_names.push_back("another");
                serial_mapping.serial_mappings.push_back(13);
                serial_mapping.types.push_back("std_msgs/String");
                serial_mapping.direction.push_back(ros2_serial_msgs::msg::SerialMapping::ROS2TOSERIAL);

                size_t serialized_size = ros2_serial_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(serial_mapping, 0);
                std::unique_ptr<uint8_t[]> data_buffer = std::unique_ptr<uint8_t[]>(new uint8_t[serialized_size]{});
                eprosima::fastcdr::FastBuffer cdrbuffer(reinterpret_cast<char *>(data_buffer.get()), serialized_size);
                eprosima::fastcdr::Cdr scdr(cdrbuffer);
                ros2_serial_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(serial_mapping, scdr);
                if (transporter->write(1, data_buffer.get(), scdr.getSerializedDataLength()) < 0)
                {
                    ::fprintf(stderr, "Failed to write dynamic response: %s\n", ::strerror(errno));
                }
            }
            else
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
}

int main(int argc, char *argv[])
{
    std::string device{};
    uint32_t baudrate = 0;
    std::string serial_protocol{"cobs"};

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

    if (device.empty())
    {
        fprintf(stderr, "No device specified\n");
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

    // We use a unique_ptr here both to make this a heap allocation and to quiet
    // non-owning pointer warnings from clang-tidy
    std::unique_ptr<uint8_t[]> data_buffer(new uint8_t[BUFFER_SIZE]);
    std::unique_ptr<uint8_t[]> data_buffer2(new uint8_t[BUFFER_SIZE]);

    while (running != 0)
    {
        // With the current configuration, topic 9 is a std_msgs/String topic.
        eprosima::fastcdr::FastBuffer cdrbuffer(reinterpret_cast<char *>(data_buffer.get()), BUFFER_SIZE);
        eprosima::fastcdr::Cdr scdr(cdrbuffer);
        scdr << "aa";
        if (transporter->write(9, data_buffer.get(), scdr.getSerializedDataLength()) < 0)
        {
            ::fprintf(stderr, "Failed to write topic %d: %s\n", 9, ::strerror(errno));
        }

        // With the current configuration, topic 12 is a std_msgs/UInt16 topic.
        eprosima::fastcdr::FastBuffer cdrbuffer2(reinterpret_cast<char *>(data_buffer2.get()), BUFFER_SIZE);
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
