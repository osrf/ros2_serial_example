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

#ifndef BOARD_H
#define BOARD_H

#include <stddef.h>
#include <stdint.h>

/* Do all board-specific initialization, including setting up clocks, LEDs, UARTs, etc. */
void board_init(void);

/* Determine whether there are any uart bytes available. */
int board_uart_byte_available(void);

/* Get the next uart byte; should only be called in board_uart_byte_available() returns true. */
uint8_t board_uart_get_byte(void);

/* Send a series of bytes to the serial port.  Returns after all bytes are queued, but not necessarily sent. */
void board_uart_send_bytes(uint8_t *bytes, size_t size);

/* Toggle the liveliness LED. */
void board_toggle_liveliness_led(void);

/* A board-specific way to show error; this could be a blinking LED or a console message, or something else. */
void board_show_error(void);

void board_toggle_spare_led(void);

#endif
