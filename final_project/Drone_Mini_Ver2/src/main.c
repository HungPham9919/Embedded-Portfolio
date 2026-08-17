#include "stdio.h"
#include "Initialize/Init_reg.h"
#include <zephyr/devicetree.h>
#include "zephyr/device.h"
#include "zephyr/kernel.h"
#include "zephyr/sys/printk.h"
#include "zephyr/usb/usb_device.h"
#include "Radio_communication/Radio_Communication.h"
#include "I2C_Progress/I2C.h"
#include "PMW3901/pmw3901.h"

int main(void)
{

    // int ret = usb_enable(NULL);
    // if(ret != 0) return 0;

    BUS_Init();

    Init_The_Config_Of_Drone();
    USART_Configuration();
    
    // Optical_Flow_Init();

    for(int i = 0; i < 2;i++) {
        GPIOC->ODR ^= (1 << 1); // toggle led
        k_msleep(100);
    }
    
    return 0;
}
