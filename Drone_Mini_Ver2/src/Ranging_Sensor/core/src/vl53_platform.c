#include "vl53l1_platform.h"
#include <zephyr/kernel.h>

// Hàm lấy tick time của Zephyr
VL53L1_Error VL53L1_GetTickCount(uint32_t *ptick_count_ms) {
    if (ptick_count_ms == NULL) {
        return VL53L1_ERROR_INVALID_PARAMS;
    }
    *ptick_count_ms = (uint32_t)k_uptime_get_32();
    return VL53L1_ERROR_NONE;
}