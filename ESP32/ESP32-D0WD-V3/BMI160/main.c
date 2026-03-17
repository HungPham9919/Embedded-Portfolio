#include "stdio.h"

void main(){
    i2c1_master_init();
    BMI160_Init();
    BMI160_Setup();
    sensitivity();

    xTaskCreate(vBMI160_Sensor_Task,"BMI_TASK",4096,NULL,5,&bmi160_task);

  // It will prints 3 angles RPY
}
