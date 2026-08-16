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

    int ret = usb_enable(NULL);
    if(ret != 0) return 0;

    BUS_Init();

    ret = drone_i2c_clearbus(dev_i2c1);
    if(ret != 0) printk("clear bus i2c1 failed");

    ret = drone_i2c_clearbus(dev_i2c3);
    if(ret != 0) printk("clear bus i2c3 failed");

    Init_The_Config_Of_Drone();
    USART_Configuration();
    
    // Optical_Flow_Init();

    DMA_I2C3_Stream();
    DMA_I2C1_Stream();
    for(int i = 0; i < 2;i++) {
        GPIOC->ODR ^= (1 << 1); // toggle led
        k_msleep(100);
    }
    
    return 0;
}
