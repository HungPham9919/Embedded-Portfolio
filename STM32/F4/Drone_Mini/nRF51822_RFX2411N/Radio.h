#ifndef RADIO_H
#define RADIO_H

#include "nrf.h"
#include "stdint.h"
#include "stdio.h"
#include "nrf51_bitfields.h"
#include "core_cm0.h"
#include "Sign.h"
#include "USART_CONFIG.h"

#define RXEN_2411 20
#define SWANT_2411 17 // SWANT = 0, RF -> ANTB || SWANT = 1, RF -> ANTA
#define MODE_2411  18
#define LED_2411N 13

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


// void nRF51822_2411_Trans_Configuration(const RF_Config_t *cfg);
// void nRF51822_2411_Send_Data(uint8_t *data);
// void nRF51822_Recv_Configurations(const RF_Config_t *cfg);

void nRF51822_2411N_Configuration(const RF_Config_t *cfg);
void Drone_To_Master(uint8_t *data, uint8_t data_len ,uint8_t drone_id, uint8_t state);
void Master_To_Drone(uint8_t drone_id,uint8_t state);
uint8_t Auto_Get_Drone_ID(void);

#endif


//void nRF51_Radio_Config_As_RX(void)
// {
//     // 1. Clock & Power
//     NRF_CLOCK->XTALFREQ = 0xFF;
//     NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
//     NRF_CLOCK->TASKS_HFCLKSTART = 1;
//     while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);

//     NRF_RADIO->POWER = 1;
//     NRF_RADIO->TASKS_DISABLE = 1;
//     while (NRF_RADIO->STATE != RADIO_STATE_STATE_Disabled);

//     NRF_RADIO->MODE = RADIO_MODE_MODE_Nrf_1Mbit;
//     NRF_RADIO->FREQUENCY = 2;

//     NRF_RADIO->PREFIX0 = swap_bits(0xCC);
//     NRF_RADIO->BASE0 = bytewise_bitswap(0xCCCCCCCC);
//     NRF_RADIO->RXADDRESSES = 1;
//     NRF_RADIO->TXADDRESS = 0;

//     NRF_RADIO->PCNF0 = 
//             (0 << RADIO_PCNF0_S0LEN_Pos) |   
//             (0 << RADIO_PCNF0_S1LEN_Pos) |     
//             (0 << RADIO_PCNF0_LFLEN_Pos);      

//     NRF_RADIO->PCNF1 =
//             (0 << RADIO_PCNF1_WHITEEN_Pos) |
//             (RADIO_PCNF1_ENDIAN_Big << RADIO_PCNF1_ENDIAN_Pos) |
//             (4 << RADIO_PCNF1_BALEN_Pos) |      // 4 bytes base + 1 byte prefix
//             (16 << RADIO_PCNF1_STATLEN_Pos) |
//             (16 << RADIO_PCNF1_MAXLEN_Pos);  

//     NRF_RADIO->CRCCNF = RADIO_CRCCNF_LEN_Two;
//     NRF_RADIO->CRCINIT = 0xFFFF;
//     NRF_RADIO->CRCPOLY = 0x11021;

//     // 6. RAM & Shorts
//     NRF_RADIO->PACKETPTR = (uint32_t)rx_buffer; 
//     NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk;

//     NRF_RADIO->EVENTS_ADDRESS = 0;
//     NRF_RADIO->EVENTS_END     = 0;
//     NRF_RADIO->EVENTS_READY   = 0;
// }


// void nRF51822_2411_Trans_Configuration(const RF_Config_t *cfg){
//     High_Clock_Enable();
//     NRF_RADIO->POWER = 1; // power on
//     NRF_RADIO->TASKS_DISABLE = 1; // Disable to config
//     while (NRF_RADIO->STATE != 0); // radio is on

//     NRF_GPIO->DIRSET = (1 << MODE_2411)|(1 << RXEN_2411)|(1 << SWANT_2411)|(1 << LED_2411N); // output
//     NRF_GPIO->OUTSET = (1 << MODE_2411); // High power. ANTA off
//     NRF_GPIO->OUTCLR = (1 << SWANT_2411); // ANTB


//     NRF_RADIO->PREFIX0 = swap_bits(0x01); 
//     NRF_RADIO->BASE0 = bytewise_bitswap(0xCCCCCCCC); 
//     NRF_RADIO->TXADDRESS = 0; // Địa chỉ vùng nhớ 0 

//     NRF_RADIO->MODE = cfg->mode; // 1Mbs 
//     NRF_RADIO->FREQUENCY = cfg->Freq; // it has 101 frequence (0-100)
//     NRF_RADIO->TXPOWER = cfg->intense_dbm; // 4dbm

//     NRF_RADIO->PCNF0 = 0; // clear -> it means LF,S0,S1 equal to 0

//     NRF_RADIO->PCNF1 = 0; // clear -> little -> LSB , whiteen disable
//     NRF_RADIO->PCNF1 = (32 << 0)|(0 << 8)|(4 << 16)|(1 << 24); 
//     // maxlength = 32, statlen = 0 ->variable, balen = 4, big edian

//     NRF_RADIO->CRCCNF = cfg->CRC_Length; // two leng byte crc
//     NRF_RADIO->CRCINIT = cfg->CRC_INIT;
//     NRF_RADIO->CRCPOLY = cfg->CRC_POLY;

//     NRF_RADIO->SHORTS = 0;
//     NRF_RADIO->SHORTS = (1UL << 0); // ready start
// }

// void nRF51822_2411_Send_Data(uint8_t *data){
//     NRF_GPIO->OUTCLR = (1 << RXEN_2411);

//     NRF_RADIO->EVENTS_READY = 0; 
//     NRF_RADIO->EVENTS_ADDRESS = 0;
//     NRF_RADIO->EVENTS_END = 0;

//     NRF_RADIO->PACKETPTR = (uint32_t)data;

//     NRF_RADIO->TASKS_TXEN = 1;
//     while (NRF_RADIO->EVENTS_END == 0);
//     NRF_RADIO->EVENTS_END = 0; // Xóa cờ

//     NRF_RADIO->TASKS_DISABLE = 1;
//     while (NRF_RADIO->STATE != 0);
// }

// void nRF51822_Recv_Configurations(const RF_Config_t *cfg) {
//     High_Clock_Enable();
//     NRF_RADIO->POWER = 1; // power on
//     NRF_RADIO->TASKS_DISABLE = 1; // Disable to config
//     while (NRF_RADIO->STATE != 0);

//     NRF_GPIO->DIRSET = (1 << MODE_2411) | (1 << RXEN_2411) | (1 << SWANT_2411) | (1 << LED_2411N);

//     NRF_GPIO->OUTSET = (1 << MODE_2411);  
//     NRF_GPIO->OUTSET = (1 << RXEN_2411); 
    

//     NRF_GPIO->OUTCLR = (1 << SWANT_2411); 

//     NRF_RADIO->PREFIX0 = swap_bits(0x01); 
//     NRF_RADIO->BASE0 = bytewise_bitswap(0xCCCCCCCC); 
//     NRF_RADIO->RXADDRESSES = 1; // 7 addresses

//     NRF_RADIO->MODE = cfg->mode; // 1Mbps 
//     NRF_RADIO->FREQUENCY = cfg->Freq; 
//     NRF_RADIO->PCNF0 = 0; 

//     NRF_RADIO->PCNF1 = 0; // clear
//     NRF_RADIO->PCNF1 = (32 << 0)|(0 << 8)|(4 << 16)|(1 << 24); 
//     // maxlength = 32, statlen = 0 -> variable, balen = 4, big edian

//     NRF_RADIO->CRCCNF = cfg->CRC_Length; 
//     NRF_RADIO->CRCINIT = cfg->CRC_INIT;
//     NRF_RADIO->CRCPOLY = cfg->CRC_POLY;

//     NRF_RADIO->PACKETPTR = (uint32_t)rx2411_buffer;

//     NRF_RADIO->SHORTS = 0;
//     NRF_RADIO->SHORTS = (1UL << 0) | (1UL << 5); 

//     // 8. Xóa sự kiện cũ và kích hoạt tiến trình Nhận
//     NRF_RADIO->EVENTS_ADDRESS = 0;
//     NRF_RADIO->EVENTS_END = 0;
//     NRF_RADIO->EVENTS_READY = 0; 
    
//     NRF_RADIO->TASKS_RXEN = 1; // Receiver
//     while(!NRF_RADIO->EVENTS_READY);
//     NRF_RADIO->TASKS_START = 1;
// }


/*
Khi phát thì TXEN high do chân VDDPA cấp. Chân RXEN phải kéo xuống low
Khi thu thì TXEN low do chân VDDPA ngắt, Chân RXEN phải kéo lên cao
Chân SWANT chọn đường xuất ra tín hiệu, SWANT = 0, RF -> ANTB || SWANT = 1, RF -> ANTA

*/