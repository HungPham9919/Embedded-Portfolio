#include "stdio.h"
#include "Initialize/Init_reg.h"
#include <stdint.h>
#include "string.h"
#include <zephyr/devicetree.h>
#include "zephyr/device.h"
#include "zephyr/kernel.h"
#include "Radio_communication/Radio_Communication.h"
#include "I2C_Progress/I2C.h"
#include "PMW3901/pmw3901.h"
#include "USB_Debugging/usb_debug.h"


int main(void)
{

    init_usb_shell();
    BUS_Init();
    Init_The_Config_Of_Drone();
    for(int i = 0; i < 2;i++) {
        GPIOC->ODR ^= (1 << 1); // toggle led
        k_msleep(100);
    }
    The_First_State();
    return 0;
}
