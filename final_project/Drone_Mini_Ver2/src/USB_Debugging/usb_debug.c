#include "usb_debug.h"
#include <zephyr/shell/shell.h>
#include "I2C_Progress/I2C.h"
#include "pmw3901.h"
#include "zephyr/usb/usb_device.h"
#include <zephyr/drivers/uart.h>
#include "stdio.h"
#include "BMI088_Library/bmi088.h"
#include "RTOS/RTOS_Init.h"

void init_usb_shell(void){
    if(usb_enable(NULL) != 0) return;
}

static int cmd_sensor_status(const struct shell *sh, size_t argc, char **argv){

    shell_print(sh, "INA226: 0x%02X", drone_sensor_addr.sensor1); // 0x40
    shell_print(sh, "AT24LC: 0x%02X", drone_sensor_addr.sensor2); // 0x50
    shell_print(sh, "BMP280: 0x%02X", drone_sensor_addr.sensor3); // 0x76

    shell_print(sh, "ACC/MAG: 0x%02X", drone_sensor_addr.sensor4); // 0x18
    shell_print(sh, "MAG_2:   0x%02X", drone_sensor_addr.sensor5); // 0x1E
    shell_print(sh, "VL53L1X: 0x%02X", drone_sensor_addr.sensor6); // 0x29
    shell_print(sh, "GYRO/IMU:0x%02X", drone_sensor_addr.sensor7); // 0x69
    shell_print(sh, "PMW3901:0x%02X || 0x%02X", product_id, inverse_product);
    shell_print(sh, "PMW3901 Error: %d", pmw3901_error_init);
    shell_print(sh, "INA Error: %d", ina_error_init);
    shell_print(sh, "BMP280 Error: %d", bmp_error_init);
    shell_print(sh, "i2c3 Error: %d", i2c3_error_count);
    shell_print(sh, "hmc5883 Error: %d", hmc_error_init);
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
