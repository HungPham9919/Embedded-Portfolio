cmake_minimum_required(VERSION 3.20.0)

list(APPEND DTS_ROOTS "${CMAKE_CURRENT_SOURCE_DIR}")

# Tự động tìm kiếm cấu hình môi trường gốc của Zephyr OS
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(Drone_Mini_Ver2)

# Ép sinh ra file compile_commands.json để VS Code hết gạch đỏ
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

zephyr_include_directories(
    src
    src/Initialize
    src/RTOS
    src/Radio_Communication
    src/BMI088_Library
    src/INA226
    src/HMC5883
    src/PMW3901
    src/BMP280
    src/PWM_Control
    src/I2C_Progress
    src/Ranging_Sensor
    src/Ranging_Sensor/core/inc
    src/Ranging_Sensor/core/src
    src/USB_Debugging
)

# Khai báo nguồn file .c
file(GLOB_RECURSE ALL_SOURCES
    src/*.c
    BMI088_Library/*.c
    RTOS/*.c
    Initialize/*.c
    PWM_Control/*.c
    Radio_Communication/*.c
    PMW3901/*.c
    BMP280/*.c
    INA226/*.c
    HMC5883/*.c
    AT24LC/*.c
    PMW_Control/*.c
    I2C_Progress/*.c
    Ranging_Sensor/*.c
    Ranging_Sensor/core/src/*.c
    USB_Debugging/*.c
)

target_sources(app PRIVATE ${ALL_SOURCES})
