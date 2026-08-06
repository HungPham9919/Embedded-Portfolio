#include "uart.h"
#include "nrf51.h"

void USART_Initialize(uint8_t TX_pin, uint8_t RX_pin){

    NRF_GPIO->PIN_CNF[TX_pin] = (1UL << 0); // Output
    NRF_GPIO->PIN_CNF[RX_pin] = (0UL << 0)|(0UL << 1)|(3UL << 2); // Input, Connect, Pull up

    NRF_UART0->PSELTXD = TX_pin;
    NRF_UART0->PSELRXD = RX_pin;

    NRF_UART0->PSELRTS = 0xFFFFFFFF; // Disconnect
    NRF_UART0->PSELCTS = 0xFFFFFFFF;

    NRF_UART0->BAUDRATE = 0x0EBEDFA4; // 921600
    NRF_UART0->CONFIG = 0;
    NRF_UART0->ENABLE = 4;

    NRF_UART0->EVENTS_RXDRDY = 0;

    // enable interrupt source
    NRF_UART0->INTENSET = (1UL << 2)|(1UL << 9); // RXRDY - Error

    NRF_UART0->TASKS_STARTTX = 1;
    NRF_UART0->TASKS_STARTRX = 1;

    NVIC_SetPriority(UART0_IRQn,2);
    NVIC_ClearPendingIRQ(UART0_IRQn);
    NVIC_EnableIRQ(UART0_IRQn);
}

static uint8_t STM32_Data[32] __attribute__((aligned(4))) __attribute__((used)) = {0};
static int nRF51822_index __attribute__((used)) = 0;
static char last_char __attribute__((used)) = 0;
static int index = 0;

void UART0_IRQHandler(void){
    if (NRF_UART0->EVENTS_ERROR) {
        NRF_UART0->EVENTS_ERROR = 0;
        NRF_UART0->ERRORSRC = NRF_UART0->ERRORSRC; 
    }

    if(NRF_UART0->EVENTS_RXDRDY){
        NRF_UART0->EVENTS_RXDRDY = 0;
        char current_char = (char)NRF_UART0->RXD;
        if (nRF51822_index < 31) {
            STM32_Data[nRF51822_index++] = current_char;
        }
        if(current_char == '\n' || current_char == '\r'){
            NVIC_SetPendingIRQ(SWI0_IRQn);
            index = nRF51822_index;
            nRF51822_index = 0;
        }
    }
}

void SWI0_Configuration(void) {
    NVIC_SetPriority(SWI0_IRQn, 3);
    NVIC_ClearPendingIRQ(SWI0_IRQn);
    NVIC_EnableIRQ(SWI0_IRQn);
}

extern uint8_t drone_id;

void SWI0_IRQHandler(void)
{
    NVIC_DisableIRQ(RADIO_IRQn);
    NRF_RADIO->SHORTS = 0;

    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE = 1;
    while (NRF_RADIO->EVENTS_DISABLED == 0); // Chờ cho đến khi Radio về trạng thái DISABLED hẳn
    NRF_RADIO->EVENTS_DISABLED = 0;

    Drone_To_Master(STM32_Data, strlen((char*)STM32_Data), drone_id, 0);

    nRF51822_index = 0; 
    memset((void*)STM32_Data, 0, 32); 
    Master_To_Drone(drone_id, 1);
}

void USART0_Send_Char(char c){
    NRF_UART0->EVENTS_TXDRDY = 0; // clear flag
    NRF_UART0->TXD = (uint8_t)c;
    while (!(NRF_UART0->EVENTS_TXDRDY));
    NRF_UART0->EVENTS_TXDRDY = 0;
}

void USART0_Send_String(char *s){
    while (*s)
    {
        USART0_Send_Char(*s++);
    }
}

void USART_Disable(void){
    NRF_UART0->EVENTS_RXDRDY = 0;
    NRF_UART0->EVENTS_TXDRDY = 0;
    NRF_UART0->ENABLE = 0;
}

void delay_ms(uint32_t ms){
    NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer;
    NRF_TIMER0->PRESCALER = 4; // 1 MHz (1 tick = 1us)
    NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit;

    NRF_TIMER0->TASKS_CLEAR = 1;
    NRF_TIMER0->TASKS_START = 1;

    while (1){
        NRF_TIMER0->TASKS_CAPTURE[0] = 1;   // copy COUNTER → CC[0]
        if (NRF_TIMER0->CC[0] >= ms * 1000){
            break;
        }
    }
    NRF_TIMER0->TASKS_STOP = 1;
}