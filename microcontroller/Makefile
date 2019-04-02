PREFIX=arm-none-eabi-
CC=$(PREFIX)gcc
LD=$(CC)
OBJCOPY=$(PREFIX)objcopy
AS=$(PREFIX)as
OPENOCD=openocd
CFLAGS=-Wall -Wextra -Wimplicit-function-declaration -Wredundant-decls -Wstrict-prototypes -Wundef -Wshadow -g -fno-common -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -MD -DSTM32F3 -I. -Ilibopencm3/include -ImicroCDR/include -ffunction-sections -Iboard -std=c11
LDFLAGS=-static -lnosys -T stm32f3discovery-ros2-serial.ld -nostartfiles -Wl,--gc-sections -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Wl,-Map=stm32f3discovery-ros2-serial.map -lc -lm
LIBOPENCM3_SRCS = libopencm3/lib/cm3/vector.c libopencm3/lib/stm32/f3/rcc.c libopencm3/lib/stm32/common/rcc_common_all.c libopencm3/lib/cm3/scb.c libopencm3/lib/cm3/nvic.c libopencm3/lib/stm32/common/gpio_common_f0234.c libopencm3/lib/stm32/common/gpio_common_all.c libopencm3/lib/stm32/common/usart_common_v2.c libopencm3/lib/stm32/common/usart_common_all.c libopencm3/lib/cm3/assert.c libopencm3/lib/stm32/common/flash_common_all.c
FREERTOS_SRCS = freertos/tasks.c freertos/list.c freertos/port.c freertos/heap_1.c
MICROCDR_SRCS = microCDR/src/c/common.c microCDR/src/c/types/array.c microCDR/src/c/types/basic.c microCDR/src/c/types/sequence.c microCDR/src/c/types/string.c
SRCS = main.c board/stm32f3discovery/board.c $(LIBOPENCM3_SRCS) $(FREERTOS_SRCS) $(MICROCDR_SRCS)
OBJS := $(filter %.o,$(SRCS:c=o) $(SRCS:s=o))

all: stm32f3discovery-ros2-serial.bin

stm32f3discovery-ros2-serial.bin: stm32f3discovery-ros2-serial.elf
	$(OBJCOPY) -Obinary stm32f3discovery-ros2-serial.elf stm32f3discovery-ros2-serial.bin

stm32f3discovery-ros2-serial.elf: $(OBJS)
	$(CC) -o stm32f3discovery-ros2-serial.elf $(OBJS) $(LDFLAGS)

flash: stm32f3discovery-ros2-serial.bin
	sudo $(OPENOCD) -f stm32-openocd.cfg -c "init" -c "reset init" -c "flash write_image erase stm32f3discovery-ros2-serial.bin 0x08000000" -c "reset run" -c "shutdown"

clean:
	rm -f *.elf *.bin *.list *.map
	find . \( -iname '*.o' -o -iname '*.d' \) -delete
