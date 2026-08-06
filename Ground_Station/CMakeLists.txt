cmake_minimum_required(VERSION 3.20.0)

# Tự động tìm kiếm cấu hình môi trường gốc của Zephyr OS
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(Ground_Station)


zephyr_include_directories(
    src
    src/SPI_Configuration_For_RF24
    src/RTOS
    src/Drone_F1
    src/Drone_F2
    src/Drone_F3
    src/Drone_F4
    src/Drone_F5
)

file(GLOB_RECURSE ALL_SOURCES
    src/*.c
    src/SPI_Configuration_For_RF24/*.c
    src/RTOS/*.c
    src/Drone_F1/*.c
    src/Drone_F2/*.c
    src/Drone_F3/*.c
    src/Drone_F4/*.c
    src/Drone_F5/*.c
)

target_sources(app PRIVATE ${ALL_SOURCES})