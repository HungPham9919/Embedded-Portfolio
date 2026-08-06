BOARD ?= adafruit_feather_stm32f405
PYTHON_EXEC ?= /usr/bin/python3.12
SDK_DIR ?= /home/hung/zephyr-sdk-0.17.0

.PHONY: all build flash clean menuconfig

all: build

# Tự động tạo thư mục build, cấu hình CMake nếu chưa có, và biên dịch bằng Ninja
build:
	@if [ ! -d "build" ]; then \
		mkdir -p build && cd build && \
		cmake -GNinja \
			-DBOARD=$(BOARD) \
			-DPython3_EXECUTABLE=$(PYTHON_EXEC) \
			-DZEPHYR_TOOLCHAIN_VARIANT=zephyr \
			-DZEPHYR_SDK_INSTALL_DIR=$(SDK_DIR) ..; \
	fi
	@cd build && ninja

# Lệnh nạp code bằng OpenOCD giữ nguyên theo đường dẫn hex của bạn
flash:
	openocd -f interface/stlink.cfg -c "transport select hla_swd" \
		-f target/stm32f4x.cfg \
		-c "reset_config none separate" \
		-c "init" \
		-c "reset init" \
		-c "flash write_image erase /home/hung/zephyrproject/Drone_Mini_Ver2/build/zephyr/zephyr.hex" \
		-c "reset run" \
		-c "shutdown"

# Xóa sạch thư mục build khi muốn dọn dẹp hệ thống
clean:
	rm -rf build