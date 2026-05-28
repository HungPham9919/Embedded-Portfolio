TARGET = main

CC = arm-none-eabi-gcc

CFLAGS = -mcpu=cortex-m0 -mthumb -g \
         -Wall -Wextra -Werror \
         -DNRF51 \
         -Imodules/nrfx/mdk \
         -Icomponents/toolchain/cmsis/include

SRC = modules/nrfx/mdk/gcc_startup_nrf51.S \
      src/main.c \
 	  src/Radio.c \
	  src/Sign.c \
      # src/syscalls.c

# LDFLAGS = -T linker.ld -Wl,--gc-sections -nostartfiles
#LDFLAGS = -T linker.ld -Wl,--gc-sections --specs=rdimon.specs -lrdimon

LDFLAGS = -T linker.ld \
          -Wl,--gc-sections \
          --specs=rdimon.specs \
          -lrdimon
all:
	mkdir -p build
	$(CC) $(CFLAGS) $(SRC) $(LDFLAGS) -o build/main.elf
	arm-none-eabi-objcopy -O ihex build/main.elf build/main.hex

clean:
	rm -rf build

flash: all
	openocd -f interface/stlink.cfg -f target/nrf51.cfg -c "init; halt; program build/main.elf verify; nrf51.cpu curstate; reset run; exit"

size: build/main.elf
	arm-none-eabi-size build/main.elf

log: all
	openocd -f interface/stlink.cfg -f target/nrf51.cfg -c "init; arm semihosting enable; program build/main.elf verify; nrf51.cpu curstate; reset run"
