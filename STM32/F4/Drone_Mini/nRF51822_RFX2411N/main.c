#include "nrf.h"
#include "stdint.h"
#include "stdio.h"
#include "nrf51_bitfields.h"
#include "core_cm0.h"
#include "Radio.h"

#define nRF51822_TX 8
#define nRF51822_RX 9

void SystemInit(void) {} 
void _exit(int status) { (void)status; while(1); }

uint8_t drone_id;
int main(void)
{
    USART_Initialize(nRF51822_TX, nRF51822_RX);
    nRF51822_2411N_Configuration(&nRF51822_cfg);
    drone_id = Auto_Get_Drone_ID();
    Master_To_Drone(drone_id,1);

    while(1){
        
    }

}

