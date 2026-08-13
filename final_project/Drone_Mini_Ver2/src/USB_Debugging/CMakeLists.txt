cmake_minimum_required(VERSION 3.20.0)

# Tự động tìm kiếm cấu hình môi trường gốc của Zephyr OS
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

target_sources(app PRIVATE usb_debug.c)

target_include_directories(app PRIVATE .)