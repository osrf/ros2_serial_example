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

#include <stddef.h>
#include <stdint.h>

#include "libopencmsis/core_cm3.h"
#include "libopencm3/cm3/scb.h"
#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/f3/nvic.h"
#include "libopencm3/stm32/rcc.h"
#include "libopencm3/stm32/usart.h"

#define __NOP() __asm__("nop")

/* This is necessary for libc to link properly; its just empty for us */
void _init(void)
{
}

static void clock_setup(void)
{
  /* It's hard to tell from this, but this sets us up for a 64MHz CPU clock. */
  rcc_clock_setup_hsi(&rcc_hsi_configs[1]);

  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOE);
  rcc_periph_clock_enable(RCC_USART1);
}

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

struct nvic_irq
{
  uint8_t irqn;
  uint8_t priority;
};

static struct nvic_irq irqs[] = {
  {
    .irqn = NVIC_USART1_EXTI25_IRQ,
    .priority = 12,
  },
};

static void nvic_setup(void)
{
  uint32_t i;

  /* see http://www.freertos.org/RTOS-Cortex-M3-M4.html */
  scb_set_priority_grouping(SCB_AIRCR_PRIGROUP_GROUP16_NOSUB);

  for (i = 0; i < ARRAY_SIZE(irqs); i++) {
    nvic_set_priority(irqs[i].irqn, irqs[i].priority);
    nvic_enable_irq(irqs[i].irqn);
  }
}

static void led_setup(void)
{
  /* Setup the liveliness LED */
  gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO11);
  gpio_set_output_options(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO11);
  gpio_clear(GPIOE, GPIO11);

  /* Setup the error LED */
  gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO9);
  gpio_set_output_options(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO9);
  gpio_clear(GPIOE, GPIO9);

  /* Setup the 'spare' LED */
  gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);
  gpio_set_output_options(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO12);
  gpio_clear(GPIOE, GPIO12);
}

#define USART1_TX_Q_SIZE 512
static uint16_t uartTxQSize = USART1_TX_Q_SIZE;
static uint8_t uartTxQueue[USART1_TX_Q_SIZE];
static uint16_t uartTxHead;
static uint16_t uartTxTail;

#define USART1_RX_Q_SIZE 512
static uint8_t uartRxQueue[USART1_RX_Q_SIZE];
static uint16_t uartRxHead;
static uint16_t uartRxTail;

static void usart_enable_tx_complete_interrupt(uint32_t usart)
{
  USART_CR1(usart) |= USART_CR1_TCIE;
}

static void usart_disable_tx_complete_interrupt(uint32_t usart)
{
  USART_CR1(usart) &= ~USART_CR1_TCIE;
}

static uint8_t usart1_send_bytes(const uint8_t *data, uint8_t num)
{
  uint8_t result = 0;

  while (num) {
    uint16_t next_head = (uartTxHead + 1) % uartTxQSize;
    if (next_head == uartTxTail) {
      break;
    }

    uartTxQueue[uartTxHead] = *data++;
    num--;
    result++;

    uartTxHead = next_head;
  }

  usart_enable_tx_complete_interrupt(USART1);

  return result;
}

void usart1_exti25_isr(void)
{
  if (usart_get_flag(USART1, USART_FLAG_RXNE)) {
    uint8_t b = usart_recv_blocking(USART1) & 0xff;
    if (((uartRxHead + 1) == uartRxTail) || ((uartRxHead == (USART1_RX_Q_SIZE - 1)) && (uartRxTail == 0))) {
      __NOP();
    } else {
      uartRxQueue[uartRxHead++] = b;
      if (uartRxHead == USART1_RX_Q_SIZE) {
        uartRxHead = 0;
      }
    }
  }

  if (usart_get_flag(USART1, USART_FLAG_TC)) {
    if (uartTxTail != uartTxHead) {
      usart_send(USART1, uartTxQueue[uartTxTail++]);
      if (uartTxTail == uartTxQSize) {
        uartTxTail = 0;
      }
    } else {
      usart_disable_tx_complete_interrupt(USART1);
    }
  }
}

static void usart1_setup(void)
{
  /* Configure USART Tx and Rx as alternate function push-pull */
  gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6);
  gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO6);

  gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO7);
  gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO7);

  gpio_set_af(GPIOB, GPIO_AF7, GPIO6);
  gpio_set_af(GPIOB, GPIO_AF7, GPIO7);

  usart_disable(USART1);

  usart_set_baudrate(USART1, 115200);
  usart_set_databits(USART1, 8);
  usart_set_stopbits(USART1, USART_STOPBITS_1);
  usart_set_parity(USART1, USART_PARITY_NONE);
  usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
  usart_set_mode(USART1, USART_MODE_TX_RX);

  /* Enable USART */
  usart_enable(USART1);

  usart_enable_rx_interrupt(USART1);
}

int board_uart_byte_available(void)
{
  return uartRxHead != uartRxTail;
}

uint8_t board_uart_get_byte(void)
{
  uint8_t byte;
  byte = uartRxQueue[uartRxTail++];
  if (uartRxTail == USART1_RX_Q_SIZE) {
    uartRxTail = 0;
  }

  return byte;
}

void board_uart_send_bytes(uint8_t *bytes, size_t size)
{
  usart1_send_bytes(bytes, size);
}

void board_toggle_liveliness_led(void)
{
  gpio_toggle(GPIOE, GPIO11);     /* LED on/off */
}

void board_show_error(void)
{
  int i;

  while (1) {
    for (i = 0; i < 1000000; i++) {  /* Wait a bit. */
      __NOP();
    }

    gpio_toggle(GPIOE, GPIO9);
  }
}

void board_toggle_spare_led(void)
{
  gpio_toggle(GPIOE, GPIO12);     /* LED on/off */
}

void board_init(void)
{
  __disable_irq();

  clock_setup();

  nvic_setup();

  led_setup();

  usart1_setup();
}
