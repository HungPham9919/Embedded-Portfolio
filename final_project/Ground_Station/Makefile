BOARD ?= nucleo_f411re
ZEPHYR_BASE ?= $(HOME)/zephyrproject/zephyr

PI_USER ?= phamthaihung9919
PI_IP   ?= 10.1.2.7
PI_PATH ?= ~/zephyrproject/Ground_Station

.PHONY: all build clean Transfer_To_Pi

all: build

build:
	west build -b $(BOARD) .

clean:
	rm -rf build

Transfer_To_Pi:
	rsync -avz --delete \
			--exclude='.cache' \
			--exclude='.vscode' \
			--exclude='.clangd' \
			--exclude='build/' \
			./ $(PI_USER)@$(PI_IP):$(PI_PATH)/
