#include "Radio.h"
#include "UART/uart.h"
#include "nrf.h"
#include "Radio.h"
#include "string.h"

static uint8_t rx2411_buffer[32] __attribute__((aligned(4))) __attribute__((used));

static uint32_t swap_bits(uint32_t inp)
{
    uint32_t i;
    uint32_t retval = 0;
    inp &= 0xFF;
    for(i = 0; i < 8; i++)
    {
        retval |= ((inp >> i) & 1) << (7 - i);
    }
    return retval;
}

static uint32_t bytewise_bitswap(uint32_t inp)
{
    return (swap_bits(inp >> 24) << 24)
        | (swap_bits(inp >> 16) << 16)
        | (swap_bits(inp >> 8 ) << 8 )
        | (swap_bits(inp));
}

const RF_Config_t nRF51822_cfg = {
    .mode = 0, // 1Mbs
    .Freq = 2, // 2402MHz
    .intense_dbm = 0x04, // 4dbm
    .CRC_Length = 2UL,
    .CRC_INIT = 0xFFFF,
    .CRC_POLY = 0x11021
};

void High_Clock_Enable(void){
    NRF_CLOCK->XTALFREQ = 0xFF;
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}

void nRF51822_2411N_Configuration(const RF_Config_t *cfg){
    High_Clock_Enable();
    NRF_RADIO->POWER = 1; // power on
    NRF_RADIO->TASKS_DISABLE = 1; // Disable to config
    while (NRF_RADIO->STATE != 0); // radio is on

    NRF_GPIO->DIRSET = (1 << MODE_2411)|(1 << RXEN_2411)|(1 << SWANT_2411)|(1 << LED_2411N); // output
    NRF_GPIO->OUTSET = (1 << MODE_2411);  // High power
    NRF_GPIO->OUTCLR = (1 << SWANT_2411); // ANTB -> CA-C03

    NRF_RADIO->BASE0 = bytewise_bitswap(0xCCCCCCCC);
    NRF_RADIO->BASE1 = bytewise_bitswap(0xCCCCCCCC);

    NRF_RADIO->MODE = cfg->mode; // 1Mbs 
    NRF_RADIO->FREQUENCY = cfg->Freq; // it has 101 frequences (0-100)
    NRF_RADIO->TXPOWER = cfg->intense_dbm; // 4dbm

    NRF_RADIO->PCNF0 = 0;
    NRF_RADIO->PCNF0 = (6 << 0)|(0 << 8)|(3 << 16);

    NRF_RADIO->PCNF1 = 0; // clear -> little -> LSB , whiteen disable
    NRF_RADIO->PCNF1 = (32 << 0)|(0 << 8)|(4 << 16)|(1 << 24); // dynamic

    NRF_RADIO->CRCCNF = (cfg->CRC_Length << 0);
    NRF_RADIO->CRCINIT = cfg->CRC_INIT;
    NRF_RADIO->CRCPOLY = cfg->CRC_POLY;

    NRF_RADIO->SHORTS = 0;
    NRF_RADIO->SHORTS = (1UL << 0)|(1UL << 1); // ready start- end - rx dis
}

void Switching_Function(int status){
    switch (status)
        {
        case 1: // receiver
            NRF_GPIO->OUTSET = (1 << RXEN_2411);  // RXEN enable
            break;
        default: // transfer
            NRF_GPIO->OUTCLR = (1 << RXEN_2411);
            break;
        }
}

void Data_Processing(uint8_t *raw_data){
    NRF_GPIO->OUTSET = (1 << LED_2411N);
    uint8_t data_len = raw_data[0] & 0x3F;
    if (data_len > 0 && data_len <= 32) {
        for(volatile int i = 0 ; i < data_len; i++){
           USART0_Send_Char(raw_data[i + 2]); 
        }
        USART0_Send_Char('\n');
    }

}

uint8_t Auto_Get_Drone_ID(void){
    uint32_t chip_id = *(volatile uint32_t *)0x10000060; // DEVICEID[0]
    switch (chip_id)
    {
    case 0x66e43b1a:
        return 1;
        break;
    case 0xdd283d1d:
        return 2;
        break;
    case 0x3efaeb68:
        return 3;
        break;
    case 0x0ad279a1:
        return 4;
        break;
    case 0x09372faf:
        return 5;  
        break;
    default:
        return 0;
        break;
    }
}

void Master_To_Drone(uint8_t drone_id, uint8_t state) { // receiver
    
    switch (drone_id)
    {
        case 1:
            NRF_RADIO->PREFIX0 = (NRF_RADIO->PREFIX0 & 0xFFFF00FF) | (swap_bits(0x01) << 8);
            NRF_RADIO->RXADDRESSES = (1 << 1); // Bật duy nhất Pipe 1
            break;
            
        case 2: 
            NRF_RADIO->PREFIX0 = (NRF_RADIO->PREFIX0 & 0xFF00FFFF) | (swap_bits(0x02) << 16);
            NRF_RADIO->RXADDRESSES = (1 << 2); // Bật duy nhất Pipe 2
            break;
            
        case 3:
            NRF_RADIO->PREFIX0 = (NRF_RADIO->PREFIX0 & 0x00FFFFFF) | (swap_bits(0x03) << 24);
            NRF_RADIO->RXADDRESSES = (1 << 3); // Bật duy nhất Pipe 3
            break;
            
        case 4: 
            NRF_RADIO->PREFIX1 = (NRF_RADIO->PREFIX1 & 0xFFFFFF00) | swap_bits(0x04);
            NRF_RADIO->RXADDRESSES = (1 << 4); // Bật duy nhất Pipe 4
            break;
            
        default: 
            NRF_RADIO->PREFIX1 = (NRF_RADIO->PREFIX1 & 0xFFFF00FF) | (swap_bits(0x05) << 8);
            NRF_RADIO->RXADDRESSES = (1 << 5); // Bật duy nhất Pipe 5
            break;
    }

    NRF_RADIO->PACKETPTR = (uint32_t)rx2411_buffer;
    NRF_RADIO->EVENTS_READY = 0; 
    NRF_RADIO->EVENTS_ADDRESS = 0;
    NRF_RADIO->EVENTS_END = 0;

    NRF_RADIO->EVENTS_DISABLED = 0; // add

    NRF_RADIO->SHORTS = (1UL << 0)|(1UL << 1); // ready start- end - rx dis
    
    NRF_RADIO->INTENCLR = 0xFFFFFFFF;
    NRF_RADIO->INTENSET = (1UL << 3) | (1UL << 1); 

    NVIC_SetPriority(RADIO_IRQn, 1); 
    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_EnableIRQ(RADIO_IRQn);

    Switching_Function(state);
    NRF_RADIO->TASKS_RXEN = 1; // Kích hoạt nhận
}

extern uint8_t drone_id;
void RADIO_IRQHandler(void)
{

    if(NRF_RADIO->EVENTS_PAYLOAD) {
        NRF_RADIO->EVENTS_PAYLOAD = 0;
    }
    // 1. Luôn xóa cờ Address để tránh kẹt ngắt
    if(NRF_RADIO->EVENTS_ADDRESS) {
        NRF_RADIO->EVENTS_ADDRESS = 0;
    }

    // 2. Xử lý khi nhận trọn vẹn gói tin (EVENTS_END nổ)
    if(NRF_RADIO->EVENTS_END)
    {
        NRF_RADIO->EVENTS_END = 0;

        if(NRF_RADIO->CRCSTATUS == 1) 
        {
            Data_Processing(rx2411_buffer);
        }
        memset(rx2411_buffer, 0, sizeof(rx2411_buffer));
        __DSB();

        while(NRF_RADIO->STATE != 0);
        NRF_RADIO->PACKETPTR = (uint32_t)rx2411_buffer;
        NRF_RADIO->TASKS_RXEN = 1; 
    }
}

void Drone_To_Master(uint8_t *data, uint8_t data_len ,uint8_t drone_id, uint8_t state){ // transfer
    static uint8_t tx_dma_buffer[34] __attribute__((aligned(4)));
    tx_dma_buffer[0] = data_len; // Byte 0: Ép trường LENGTH
    tx_dma_buffer[1] = 0; // Byte 1: Ép trường S1 = 0

    memcpy(&tx_dma_buffer[2], data, data_len);


    NRF_RADIO->PREFIX0 = (NRF_RADIO->PREFIX0 & 0xFFFFFF00) | (swap_bits(drone_id));
    NRF_RADIO->TXADDRESS = 0;

    Switching_Function(state);
    NRF_RADIO->EVENTS_ADDRESS = 0;
    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->EVENTS_READY = 0; 

    NRF_RADIO->PACKETPTR = (uint32_t)tx_dma_buffer;
    
    NRF_RADIO->TASKS_TXEN = 1;
    while (NRF_RADIO->EVENTS_READY == 0); 
    NRF_RADIO->EVENTS_READY = 0;

    NRF_RADIO->TASKS_START = 1;      
    while (NRF_RADIO->EVENTS_END == 0);
    NRF_RADIO->EVENTS_END = 0; 

    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE = 1;

    while(!NRF_RADIO->EVENTS_DISABLED);

    NRF_RADIO->EVENTS_DISABLED = 0;
}
