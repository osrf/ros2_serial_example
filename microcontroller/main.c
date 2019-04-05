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

#include "board/board.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ucdr/microcdr.h"

void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName)
{
  /* This function will get called if a task overflows its stack.   If the
     parameters are corrupt then inspect pxCurrentTCB to find which was the
     offending task. */

  (void)pxTask;
  (void)pcTaskName;

  board_show_error();
}

static void blinky_task(void *arg)
{
  (void)arg;

  while (1) {
    board_toggle_liveliness_led();
    vTaskDelay(MS_TO_TICKS(1000));
  }
}

// This function decodes length bytes of data at the location pointed to by
// ptr, writing the output to the location pointed to by dst.  The amount of
// memory needed by dst is guaranteed to be <= length.
//
// Returns the length of the decoded data.
size_t cobs_unstuff_data(const uint8_t *ptr, size_t length, uint8_t *dst)
{
    const uint8_t *start = dst;
    const uint8_t *end = ptr + length;
    uint8_t code = 0xff;
    uint8_t copy = 0;

    for (; ptr < end; copy--)
    {
        if (copy != 0)
        {
            *dst++ = *ptr++;
        }
        else
        {
            if (code != 0xff)
            {
                *dst++ = 0;
            }
            copy = code = *ptr++;
            if (code == 0)
            {
                break;  // source length too long
            }
        }
    }

    return dst - start;
}

/** CRC table for the CRC-16. The poly is 0x8005 (x^16 + x^15 + x^2 + 1) */
static uint16_t const crc16_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

uint16_t crc16_byte(uint16_t crc, uint8_t data)
{
  return (crc >> 8U) ^ crc16_table[(uint8_t)crc ^ data];
}

uint16_t crc16(uint8_t const *buffer, size_t len)
{
  uint16_t crc = 0;

  while ((len--) != 0) {
    crc = crc16_byte(crc, *buffer++);
  }

  return crc;
}

#define MSG_BUFFER_SIZE 512
static uint8_t msgBuffer[MSG_BUFFER_SIZE];
static uint16_t msgBufferPtr;

// When unstuffing COBS, we know that it can't be larger than the original.
static uint8_t cobsBuffer[MSG_BUFFER_SIZE];

typedef uint8_t topic_id_size_t;
struct __attribute__((packed)) COBSHeader
{
  topic_id_size_t topic_ID;
  uint8_t payload_len_h;
  uint8_t payload_len_l;
  uint8_t crc_h;
  uint8_t crc_l;
};

static void serial_task(void *arg)
{
  uint8_t byte;

  (void)arg;

  while (1) {
    while (board_uart_byte_available()) {
      // data available in the RX queue
      byte = board_uart_get_byte();

      msgBuffer[msgBufferPtr++] = byte;

      if (byte == 0x0) {
        // OK, we saw a full COBS message.  Let's try to decode.
        // The COBS header is a minimum of 5 bytes.  If we don't have that, this
        // is definitely not a valid message
        if (msgBufferPtr > sizeof(struct COBSHeader)) {
          size_t unstuffed_size = cobs_unstuff_data(msgBuffer, msgBufferPtr, cobsBuffer);
          if (unstuffed_size >= sizeof(struct COBSHeader)) {
            struct COBSHeader *header = (struct COBSHeader *)cobsBuffer;
            uint16_t payload_len = (uint16_t)header->payload_len_h << 8U | header->payload_len_l;
            if ((unstuffed_size - sizeof(struct COBSHeader)) >= payload_len) {
              uint16_t read_crc = (uint16_t)header->crc_h << 8U | header->crc_l;
              uint16_t calc_crc = crc16(&cobsBuffer[0] + sizeof(struct COBSHeader), payload_len);
              if (read_crc == calc_crc) {
                // Valid buffer!  We can CDR unencode it now.  Note that we
                // reuse the msgBuffer, since we no longer need the original contents.
                ucdrBuffer reader;
                ucdr_init_buffer(&reader, cobsBuffer + sizeof(struct COBSHeader), payload_len);
                ucdr_deserialize_string(&reader, (char *)msgBuffer, MSG_BUFFER_SIZE);
              }
            }
          }
        }
        msgBufferPtr = 0;
      }

      if (msgBufferPtr == MSG_BUFFER_SIZE) {
        // We overflowed the message buffer; this can't be a valid message, so
        // throw away what we have and try again.
        msgBufferPtr = 0;
      }

      //byte += 1;

      //board_uart_send_bytes(&byte, 1);
      board_toggle_spare_led();
    }

    vTaskDelay(MS_TO_TICKS(1));
  }
}

int main(void)
{
  board_init();

  // Empirically, the smallest stack we can have for blinky is 37
  if (xTaskCreate(blinky_task, "blinky", 37, NULL,
		  tskIDLE_PRIORITY + 1, NULL) != pdTRUE) {
    while(1);
  }

  if (xTaskCreate(serial_task, "tx", 100, NULL,
                  tskIDLE_PRIORITY + 2, NULL) != pdTRUE) {
    while(1);
  }

  vTaskStartScheduler();

  while (1);

  return 0;
}
