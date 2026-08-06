BOARD ?= nucleo_f411re
ZEPHYR_BASE ?= $(HOME)/zephyrproject/zephyr

.PHONY: all build flash clean menuconfig

all: build

build:
	west build -b $(BOARD) .

clean:
	rm -rf build