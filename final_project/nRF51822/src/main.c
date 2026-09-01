#include "nrf51.h"
#include "UART/uart.h"
#include "Radio/Radio.h"

#define nRF51822_TX 8
#define nRF51822_RX 9

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

uint8_t drone_id = 0;

int main(void){
    // Khởi tạo các ngoại vi
    drone_id = Auto_Get_Drone_ID();

    USART_Initialize(nRF51822_TX, nRF51822_RX);
    nRF51822_2411N_Configuration(&nRF51822_cfg);

    // LED đẫ set bên nRF51822_Config rồi

    uint8_t data[] = "HELLO";
    uint8_t data1[] = "HELL_NO";

    while(1){
        NRF_GPIO->OUTSET = (1 << LED_2411N);
        Drone_To_Master(data, sizeof(data), drone_id, 0);
        k_msleep(200);

        NRF_GPIO->OUTCLR = (1 << LED_2411N);
        Drone_To_Master(data1, sizeof(data1), drone_id, 0);
        k_msleep(200);
    }
    return 0;
}
