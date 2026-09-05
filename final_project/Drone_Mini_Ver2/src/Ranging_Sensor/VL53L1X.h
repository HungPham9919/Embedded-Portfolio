#ifndef VL53L1X_H
#define VL53L1X_H

#include "zephyr/devicetree.h"
#include "zephyr/kernel.h"
#include "stdio.h"
#include "stdint.h"

#include "I2C_Progress/I2C.h"
#include <stdint.h>
#include "stdio.h"
#include "RTOS/RTOS_Init.h"


#include "vl53l1_api.h"
#include "vl53l1_api_core.h"
#include "vl53l1_def.h"
#include "vl53l1_error_codes.h"
#include "vl53l1_ll_device.h"
#include "vl53l1_platform.h"

extern VL53L1_Dev_t vl53_dev_struct;
extern VL53L1_DEV vl53_dev;
extern volatile uint16_t VL53_ID;
VL53L1_Error VL53L1_Init(VL53L1_DEV Dev);

#endif
