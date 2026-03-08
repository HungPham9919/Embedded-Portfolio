#include <stdio.h>
#include "Soil_Sensor.h"

void app_main(void) {
    UART_CONFIG();
    // 0x02 is slave id, the second of parameters u can see in enum function.
    Read_Sensor(0x02,MOISTURE_CONTENT); 
}
