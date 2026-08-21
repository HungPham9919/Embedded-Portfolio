#include "stdio.h"
#include "Initialize/Init_reg.h"
#include <zephyr/devicetree.h>
#include "zephyr/kernel.h"
#include "zephyr/usb/usb_device.h"
#include "Radio_communication/Radio_Communication.h"
#include "I2C_Addr_Scanner/Check_Address_I2C1.h"
#include "PMW3901/pmw3901.h"
#include "USB_Debugging/usb_debug.h"

int main(void)
{

    init_usb_shell();
    BUS_Init();

    I2C3_ClearBus();
	I2C1_ClearBus();

    I2C3_Initialized();
    I2C1_Initialized();

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
