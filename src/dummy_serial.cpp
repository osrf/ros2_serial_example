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

#include <csignal>
#include <cstdio>
#include <cstring>
#include <memory>
#include <thread>

#include <termios.h>
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

void read_thread_func(Transporter * transporter)
{
    char data_buffer[BUFFER_SIZE] = {};
    ssize_t length = 0;
    topic_id_size_t topic_ID;

    while (running)
    {
        // Process data coming over serial
        while ((length = transporter->read(&topic_ID, data_buffer, BUFFER_SIZE)) > 0)
        {
            fprintf(stderr, "Topic ID: %d, data: ", topic_ID);
            for (ssize_t i = 0; i < length; ++i)
            {
                fprintf(stderr, "0x%x ", data_buffer[i]);
            }
            fprintf(stderr, "\n");
        }
    }
}

int main(int argc, char *argv[])
{
    char device[64] = "/dev/ttyACM0";

    int ch;
    while ((ch = ::getopt(argc, argv, "d:h")) != EOF)
    {
        switch (ch)
        {
        case 'd':
            if (optarg != nullptr)
            {
                ::strcpy(device, optarg);
            }
            break;
        case 'h':
            usage(argv[0]);
            return 0;
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

    std::unique_ptr<Transporter> transporter = std::make_unique<UARTTransporter>(device, B115200, 1);

    if (transporter->init() < 0)
    {
        return 1;
    }

    ::signal(SIGINT, signal_handler);

    std::thread read_thread(read_thread_func, transporter.get());

    size_t headlen = transporter->get_header_length();

    while (running)
    {
        char data_buffer[BUFFER_SIZE] = {};
        eprosima::fastcdr::FastBuffer cdrbuffer(&data_buffer[headlen], sizeof(data_buffer) - headlen);
        eprosima::fastcdr::Cdr scdr(cdrbuffer);
        scdr << "aa";
        transporter->write(9, data_buffer, scdr.getSerializedDataLength());

        char data_buffer2[BUFFER_SIZE] = {};
        eprosima::fastcdr::FastBuffer cdrbuffer2(&data_buffer2[headlen], sizeof(data_buffer2) - headlen);
        eprosima::fastcdr::Cdr scdr2(cdrbuffer2);
        scdr2 << "bb";
        transporter->write(12, data_buffer2, scdr2.getSerializedDataLength());

        // every 100 milliseconds
        ::usleep(100000);
    }

    read_thread.join();

    transporter->close();

    return 0;
}
