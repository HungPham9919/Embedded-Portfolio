cmake_minimum_required(VERSION 3.20.0)

# Tự động tìm kiếm cấu hình môi trường gốc của Zephyr OS
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(nRF51822)

# Ép sinh ra file compile_commands.json để VS Code hết gạch đỏ
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")


zephyr_include_directories(
    src
    src/UART
    src/Radio
)

# Khai báo nguồn file .c
file(GLOB_RECURSE ALL_SOURCES
    src/*.c
    src/UART/*.c
    src/Radio/*.c
)

target_sources(app PRIVATE ${ALL_SOURCES})
