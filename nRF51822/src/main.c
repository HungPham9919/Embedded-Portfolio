#include "nrf51.h"
#include "uart.h"
#include "Radio.h"

#define nRF51822_TX 8
#define nRF51822_RX 9

#define STM32_RST 29
#define STM32_BOOT 28

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

int main(void){
    NRF_GPIO->DIRSET = (1 << LED_2411N);
    NRF_GPIO->OUTSET = (1 << LED_2411N);

    while (1) {
        NRF_GPIO->OUT ^= (1 << LED_2411N);
        k_msleep(500);
    }
    return 0;
}


    // USART_Initialize(nRF51822_TX, nRF51822_RX);
    // SWI0_Configuration();
    // nRF51822_2411N_Configuration(&nRF51822_cfg);
    // drone_id = Auto_Get_Drone_ID();
    // Master_To_Drone(drone_id,1);

    // uint8_t data[] = "HELLO";
    // uint8_t data1[] = "HELL_NO";
    // NRF_GPIO->OUTCLR = (1 << STM32_RST);
    // NRF_GPIO->OUTSET = (1 << LED_2411N);
    // while(1){
    //     // __WFI();
    //     // Drone_To_Master(data,sizeof(data),drone_id,0);
    //     // delay_ms(500);
    //     // Drone_To_Master(data1,sizeof(data1),drone_id,0);
    //     // delay_ms(500);
    //     // NRF_GPIO->OUT ^= (1 << LED_2411N);



    //     // NRF_GPIO->OUTCLR = (1 << STM32_RST);
    //     // delay_ms(500);

    //     // NRF_GPIO->OUTSET = (1 << STM32_RST);
    //     // delay_ms(1000);
    //     // NRF_GPIO->OUT ^= (1 << LED_2411N);
    // }