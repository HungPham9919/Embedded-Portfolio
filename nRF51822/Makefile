BOARD ?= nrf51_ble400
ZEPHYR_BASE ?= $(HOME)/zephyrproject/zephyr

.PHONY: all build flash clean menuconfig

all: build

build:
	west build -b $(BOARD) .

flash:
	openocd -f interface/stlink.cfg -f target/nrf51.cfg -c "init; halt; program build/zephyr/zephyr.hex verify; reset run; exit"

# Xóa sạch thư mục build
clean:
	rm -rf build