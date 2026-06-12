#include "USART_CONFIG.h"

void USART_Initialize(uint8_t TX_pin, uint8_t RX_pin){

    NRF_GPIO->PIN_CNF[TX_pin] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);

    NRF_GPIO->PIN_CNF[RX_pin] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) | 
                                (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) | 
                                (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);

    NRF_UART0->PSELTXD = TX_pin;
    NRF_UART0->PSELRXD = RX_pin;

    NRF_UART0->PSELRTS = 0xFFFFFFFF;
    NRF_UART0->PSELCTS = 0xFFFFFFFF;

    NRF_UART0->BAUDRATE = 0x0EBEDFA4; // 921600
    NRF_UART0->CONFIG = 0;
    NRF_UART0->ENABLE = 4;

    NRF_UART0->EVENTS_RXDRDY = 0;

    // enable interrupt source
    NRF_UART0->INTENSET =
        UART_INTENSET_RXDRDY_Msk |
        UART_INTENSET_ERROR_Msk;

    NRF_UART0->TASKS_STARTTX = 1;
    NRF_UART0->TASKS_STARTRX = 1;

    NVIC_SetPriority(UART0_IRQn,2);
    NVIC_ClearPendingIRQ(UART0_IRQn);
    NVIC_EnableIRQ(UART0_IRQn);
}

// STM32 -> STATE - LED32 sáng || FLY - LED32 tắt

static uint8_t STM32_Data[32] __attribute__((aligned(4))) __attribute__((used)) = {0};
static int nRF51822_index __attribute__((used)) = 0;
static char last_char __attribute__((used)) = 0;

void UART0_IRQHandler(void){
    // 1. Xóa cờ lỗi (Không reset mảng để tránh đứt chuỗi do nhiễu)
    if (NRF_UART0->EVENTS_ERROR) {
        NRF_UART0->EVENTS_ERROR = 0;
        NRF_UART0->ERRORSRC = NRF_UART0->ERRORSRC; 
    }

    // 2. Nhận dữ liệu
    if(NRF_UART0->EVENTS_RXDRDY){
        NRF_UART0->EVENTS_RXDRDY = 0;
        char current_char = (char)NRF_UART0->RXD;
        if (nRF51822_index < 31) {
            STM32_Data[nRF51822_index++] = current_char;
            NRF_GPIO->OUT ^= (1 << LED_2411N); // chop tat
        }
        if (last_char == 'O' && current_char == 'K') {
            
            // NRF_GPIO->OUT ^= (1 << LED_2411N); // Bật LED
            NVIC_SetPendingIRQ(SWI0_IRQn);

            // nRF51822_index = 0; 
            // memset((void*)STM32_Data, 0, 32); 
        }

        // Reset index nếu gặp \n (đề phòng trượt chữ OK)
        if (current_char == '\n' || current_char == '\r') {
            nRF51822_index = 0;
        }
        
        last_char = current_char;
    }
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

void SWI0_Configuration(void) {
    NVIC_SetPriority(SWI0_IRQn, 1);
    NVIC_ClearPendingIRQ(SWI0_IRQn);
    NVIC_EnableIRQ(SWI0_IRQn);
}

extern uint8_t drone_id;

void SWI0_IRQHandler(void)
{
    uint8_t ack_payload_num[9][6] = {
            "DONE1",
            "DONE2",
            "DONE3",
            "DONE4",
            "DONE5",
            "DONE6",
            "DONE7",
            "DONE8",
            "DONE9"
        };

    NVIC_DisableIRQ(RADIO_IRQn);
    NRF_RADIO->SHORTS = 0; // add
    
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE = 1;
    while (NRF_RADIO->EVENTS_DISABLED == 0); // Chờ cho đến khi Radio về trạng thái DISABLED hẳn
    NRF_RADIO->EVENTS_DISABLED = 0;

    for (int i = 0; i < 9; i++) {
        // Gọi hàm cấu hình TX và kích phát
        Drone_To_Master(ack_payload_num[i], strlen((char*)ack_payload_num), drone_id, 0);
        NRF_GPIO->OUT ^= (1 << LED_2411N);
        delay_ms(20);
    }

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