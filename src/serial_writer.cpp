#include <cstdio>
#include <cstring>
#include <memory>

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

    std::unique_ptr<Transporter> transporter = std::make_unique<UARTTransporter>(device, B115200, 0);

    if (transporter->init() < 0)
    {
        return 1;
    }

    char data_buffer[BUFFER_SIZE] = {};
    eprosima::fastcdr::FastBuffer cdrbuffer(&data_buffer[9], sizeof(data_buffer) - 9);
    eprosima::fastcdr::Cdr scdr(cdrbuffer);
    scdr << "aa";
    transporter->write(9, data_buffer, scdr.getSerializedDataLength());

    char data_buffer2[BUFFER_SIZE] = {};
    eprosima::fastcdr::FastBuffer cdrbuffer2(&data_buffer2[9], sizeof(data_buffer2) - 9);
    eprosima::fastcdr::Cdr scdr2(cdrbuffer2);
    scdr2 << "bb";
    transporter->write(12, data_buffer2, scdr2.getSerializedDataLength());

    transporter->close();

    return 0;
}
