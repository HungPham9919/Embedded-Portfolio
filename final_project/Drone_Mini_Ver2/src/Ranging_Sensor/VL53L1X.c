#include "VL53L1X.h"


VL53L1_Dev_t vl53_dev_struct;
VL53L1_DEV vl53_dev = &vl53_dev_struct;


volatile uint16_t VL53_ID = 0;
VL53L1_Error VL53L1_Init(VL53L1_DEV Dev) {
    VL53L1_Error status = VL53L1_ERROR_NONE;
    uint16_t sensor_id = 0;

    if (Dev == NULL) Dev = vl53_dev;
    Dev->devAddr = 0x29;

    // 1. Check ID
    status = VL53L1_RdWord(Dev, 0x010F, &sensor_id);
    if (status != VL53L1_ERROR_NONE || sensor_id != 0xEACC) return VL53L1_ERROR_CONTROL_INTERFACE;
    VL53_ID = sensor_id;
    // 2. DataInit & StaticInit
    status = VL53L1_DataInit(Dev);
    if (status != VL53L1_ERROR_NONE) return status;

    status = VL53L1_StaticInit(Dev);
    if (status != VL53L1_ERROR_NONE) return status;

    // 3. Set Distance Mode SHORT
    status = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_SHORT);
    if (status != VL53L1_ERROR_NONE) return status;

    // 4. TIMING BUDGET & PERIOD (CHÚ Ý TỶ LỆ NÀY)
    // Budget = 20ms (20000us) thì Period BẮT BUỘC phải lớn hơn Budget + 25ms -> Chọn 50ms hoặc 33ms!
    status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev, 20000);
    if (status != VL53L1_ERROR_NONE) return status;

    status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, 33); // Tối thiểu 33ms đến 50ms cho Autonomous!
    if (status != VL53L1_ERROR_NONE) return status;

    return status;
}
