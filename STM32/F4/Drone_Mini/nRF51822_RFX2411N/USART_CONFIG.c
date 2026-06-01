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
static char last_char __attribute__((used)) = 0; // Dùng để bắt cặp 'O' và 'K'

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

        // Lưu vào mảng của bạn (đã được bảo vệ bởi attribute)
        if (nRF51822_index < 31) {
            STM32_Data[nRF51822_index++] = current_char;
        }

        // Logic "Bắt quả tang" tức thì: Thấy 'O' đi liền 'K' là bật LED luôn
        if (last_char == 'O' && current_char == 'K') {
            
            NRF_GPIO->OUT ^= (1 << LED_2411N); // Bật LED
            
            // Xóa mảng chuẩn bị cho chu kỳ sau
            nRF51822_index = 0; 
            memset((void*)STM32_Data, 0, 32); 
        }

        // Reset index nếu gặp \n (đề phòng trượt chữ OK)
        if (current_char == '\n' || current_char == '\r') {
            nRF51822_index = 0;
        }
        
        last_char = current_char;
    }
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