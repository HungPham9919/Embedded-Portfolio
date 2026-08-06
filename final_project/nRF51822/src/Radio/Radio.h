#ifndef RADIO_H
#define RADIO_H

#include "zephyr/kernel.h"
#include "stdio.h"
#include "stdint.h"

#include "nrf.h"
#include "stdint.h"
#include "stdio.h"

#define RXEN_2411 20
#define SWANT_2411 17 // SWANT = 0, RF -> ANTB || SWANT = 1, RF -> ANTA
#define MODE_2411  18
#define LED_2411N 10

#define Receiver 1
#define Transmitter 0

typedef struct {
    uint8_t mode;
    uint8_t Freq;
    uint8_t intense_dbm;
    uint32_t CRC_INIT;
    uint32_t CRC_POLY;
    uint8_t CRC_Length;
} RF_Config_t;

extern const RF_Config_t nRF51822_cfg;
void nRF51822_Recv_Configuration(const RF_Config_t *cfg);
void nRF51822_Trans_Configuration(const RF_Config_t *cfg);
void nRF51822_Send_Data(uint8_t *data);

void nRF51822_2411N_Configuration(const RF_Config_t *cfg);
void Drone_To_Master(uint8_t *data, uint8_t data_len ,uint8_t drone_id, uint8_t state);
void Master_To_Drone(uint8_t drone_id,uint8_t state);
uint8_t Auto_Get_Drone_ID(void);

#endif