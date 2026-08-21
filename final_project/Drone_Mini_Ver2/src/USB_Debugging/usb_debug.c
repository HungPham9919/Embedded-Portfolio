#include "usb_debug.h"
#include <zephyr/shell/shell.h>
#include "I2C.h"
#include "pmw3901.h"
#include "zephyr/usb/usb_device.h"
#include <zephyr/drivers/uart.h>
#include "stdio.h"
#include "BMI088_Library/bmi088.h"

void init_usb_shell(void){
    // const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart));
    if(usb_enable(NULL) != 0) return;
    // uint32_t dtr = 0;
    // while(!dtr){
    //     uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
    //     k_msleep(100);
    // }
}

static int cmd_sensor_status(const struct shell *sh, size_t argc, char **argv){
    shell_print(sh, "ACC/MAG: 0x%02X", drone_sensor_addr.sensor4); // 0x18
    shell_print(sh, "MAG_2:   0x%02X", drone_sensor_addr.sensor5); // 0x1E
    shell_print(sh, "VL53L1X: 0x%02X", drone_sensor_addr.sensor6); // 0x29
    shell_print(sh, "GYRO/IMU:0x%02X", drone_sensor_addr.sensor7); // 0x69
    return 0;
}


static int cmd_sensor_calib_data(const struct shell *sh, size_t argc, char **argv) {
    shell_print(sh, "Gyro Off Z: %.2f", (double)bmi088_offset.gyro_offset_z);
    return 0;
}

// Tạo cây lệnh con (Sub-commands)
SHELL_STATIC_SUBCMD_SET_CREATE(sub_sensor,
    SHELL_CMD(status, NULL, "Print sensor status registers", cmd_sensor_status),
    SHELL_CMD(calib, NULL, "Print calibration offsets", cmd_sensor_calib_data),
    SHELL_SUBCMD_SET_END
);

// Đăng ký lệnh gốc "sensor"
SHELL_CMD_REGISTER(sensor, &sub_sensor, "Sensor management commands", NULL);
SHELL_CMD_REGISTER(sensor_status, NULL, "Print sensor status registers", cmd_sensor_status);
SHELL_CMD_REGISTER(sensor_calib, NULL, "Print calibration offsets", cmd_sensor_calib_data);
