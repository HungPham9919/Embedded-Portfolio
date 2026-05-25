/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define W_REGISTER    0x20
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2

uint8_t nRF_SPI_SEND(uint8_t data){
	while(!(SPI1->SR & (1 << 1))); // txe
	SPI1->DR = data;
	while(!(SPI1->SR & (1 << 0))); // rxne
	return (uint8_t)SPI1->DR;
}

uint8_t nRF2_SPI_SEND(uint8_t data){
    while(!(SPI2->SR & (1 << 1))); 
    *(volatile uint8_t *)&SPI2->DR = data; 
    while(!(SPI2->SR & (1 << 0))); 
    return *(volatile uint8_t *)&SPI2->DR;
}

void nRF2_Write_Reg(uint8_t reg, uint8_t data){
    GPIOA->BSRR = (1 << (8 + 16)); // CSN-PA8 Low
    nRF2_SPI_SEND(0x20 | reg);     // W_REGISTER | reg
    nRF2_SPI_SEND(data);
    GPIOA->BSRR = (1 << 8);        // CSN-PA8 High
}

uint8_t nRF2_Read_Reg(uint8_t reg){
    uint8_t res;
    GPIOA->BSRR = (1 << (8 + 16)); // CSN-PA8 Low
    nRF2_SPI_SEND(reg);            // R_REGISTER | reg
    res = nRF2_SPI_SEND(0xFF);
    GPIOA->BSRR = (1 << 8);        // CSN-PA8 High
    return res;
}
uint8_t my_addr[5] = {0x01, 0xCC, 0xCC, 0xCC, 0xCC};

void nRF2_Config_Receiver(void){
GPIOB->BSRR = (1 << (12 + 16)); // CE Low
    
    nRF2_Write_Reg(0x03, 0x03); // 5 bytes address width
    nRF2_Write_Reg(0x00, 0x0F); // Power up, PRX
    nRF2_Write_Reg(0x01, 0x00); // T?t EN_AA
    nRF2_Write_Reg(0x05, 0x02); // Channel 2
    nRF2_Write_Reg(0x06, 0x01); // 1Mbps
    nRF2_Write_Reg(0x11, 16);   // Payload 16 bytes
		nRF2_Write_Reg(0x1C, 0x00); // Disable Dynamic Payload
    nRF2_Write_Reg(0x1D, 0x00); // Disable ALL features

    // GÁN Ð?A CH? NH?N CHO CON THU (nRF2)
    GPIOA->BSRR = (1 << (8 + 16)); // CSN2 Low
    nRF2_SPI_SEND(0x20 | 0x0A);    // RX_ADDR_P0
    for(int i=0; i < 5; i++) nRF2_SPI_SEND(my_addr[i]);
    GPIOA->BSRR = (1 << 8);        // CSN2 High

    GPIOB->BSRR = (1 << 12); // CE High
}

void nRF2_Receive_Payload(uint8_t *pData, uint16_t size) {
    GPIOA->BSRR = (1 << (8 + 16)); // CSN Low
    nRF2_SPI_SEND(0x61);           // L?nh R_RX_PAYLOAD
    for(int i=0; i<size; i++) {
        pData[i] = nRF2_SPI_SEND(0xFF);
    }
    GPIOA->BSRR = (1 << 8);        // CSN High
    
    // Xóa c? RX_DR (bit 6) sau khi d?c xong
    nRF2_Write_Reg(0x07, (1 << 6)); 
}

void nRF_Write_Reg(uint8_t reg, uint8_t data){
	GPIOA->BSRR = (1 << (3 + 16)); 
	nRF_SPI_SEND(W_REGISTER | reg);
	nRF_SPI_SEND(data);
	GPIOA->BSRR = (1 << 3);
}

void nRF24L01_Config(void){
    GPIOA->BSRR = (1 << (4 + 16)); // CE Low
    
    nRF_Write_Reg(0x03, 0x03); // SETUP_AW: 5bytes address
    nRF_Write_Reg(0x00, 0x0E); // Power up, TX mode
		nRF_Write_Reg(0x02, 0x01); // enable pipe 0
    nRF_Write_Reg(0x01, 0x00); // Auto-Ack off
    nRF_Write_Reg(0x05, 0x02); // Channel 2
    nRF_Write_Reg(0x06, 0x01); // 1Mbps
		nRF_Write_Reg(0x1C, 0x00); // Disable Dynamic Payload
    nRF_Write_Reg(0x1D, 0x00); // Disable ALL features
		nRF_Write_Reg(0x11, 16);
    
    // Gán d?a ch? phát (TX_ADDR) - G?i LSB tru?c
    GPIOA->BSRR = (1 << (3 + 16)); 
    nRF_SPI_SEND(W_REGISTER | 0x10); 
    nRF_SPI_SEND(0x01); 
	  nRF_SPI_SEND(0xCC); 
		nRF_SPI_SEND(0xCC); 
		nRF_SPI_SEND(0xCC); 
		nRF_SPI_SEND(0xCC);
    GPIOA->BSRR = (1 << 3);

    // Gán d?a ch? nh?n Pipe 0 (RX_ADDR_P0) ph?i gi?ng h?t TX_ADDR
    GPIOA->BSRR = (1 << (3 + 16)); 
    nRF_SPI_SEND(W_REGISTER | 0x0A); 
    nRF_SPI_SEND(0x01);
		nRF_SPI_SEND(0xCC);
		nRF_SPI_SEND(0xCC);
		nRF_SPI_SEND(0xCC);
		nRF_SPI_SEND(0xCC);
    GPIOA->BSRR = (1 << 3);
}

uint8_t nRF_Read_Reg(uint8_t reg){
    uint8_t command = reg; // L?nh d?c thanh ghi (0x00 | reg)
    uint8_t res;

    GPIOA->BSRR = (1 << (3 + 16)); // CSN = 0 (B?t d?u phiên SPI)
    
    nRF_SPI_SEND(command);         // G?i d?a ch? thanh ghi mu?n d?c
    res = nRF_SPI_SEND(0xFF);      // G?i byte gi? d? nh?n v? giá tr? thanh ghi
    
    GPIOA->BSRR = (1 << 3);        // CSN = 1 (K?t thúc phiên SPI)
    
    return res;
}

void nRF_Transmit(uint8_t *pData, uint16_t size){
	
	nRF2_Write_Reg(0x07, 0x70);
    // 1. Flush TX trước để đảm bảo sạch sẽ
    GPIOA->BSRR = (1 << (3 + 16)); 
    nRF_SPI_SEND(0xE1); 
    GPIOA->BSRR = (1 << 3);

    // 2. Nạp Payload (không dùng DMA dể test)
    GPIOA->BSRR = (1 << (3 + 16));
    nRF_SPI_SEND(0xA0); // W_TX_PAYLOAD
    for(int i=0; i<size; i++) {
        nRF_SPI_SEND(pData[i]);
    }
    GPIOA->BSRR = (1 << 3); // CSN High

    // 3. Xung CE d? kích phát
    GPIOA->BSRR = (1 << 4);        // CE High
    HAL_Delay(1);
    GPIOA->BSRR = (1 << (4 + 16)); // CE Low
}

volatile uint8_t status, config;
volatile int count = 0;

volatile uint16_t status2, config2;
uint8_t rx_buffer[16];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
	RCC->APB1ENR |= (1 << 14); // SPI2
	RCC->AHBENR |= (1 << 0); // DMA1
  	RCC->APB2ENR |= (1 << 2)|(1 << 12)|(1 << 3); // GPIOA - SPI1
 	GPIOA->CRL &= ~(0x0F << 20) &~(0x0F << 24) &~(0x0F << 28) &~(0x0F << 16) &~(0x0F << 12);
 	GPIOA->CRL |= (0x03 << 16)|(0x03 << 12); // output push pull
  	GPIOA->CRL |= (0x0B << 20)|(0x08 << 24)|(0x0B << 28); // SCK - MOSI = AF-PP || MISO - input floating
	
	GPIOB->CRH &= ~(0x0FFF << 16);
	GPIOB->CRH |= (0x03 << 16);        // PB12 (CE): Output push-pull 50MHz
	GPIOB->CRH |= (0x0B << 20);        // PB13 (SCK): Alternate function push-pull
	GPIOB->CRH |= (0x04 << 24);        // PB14 (MISO): Input floating (ho?c 0x08 Input pull-up)
	GPIOB->CRH |= (0x0B << 28);        // PB15 (MOSI): Alternate function push-pull

	GPIOA->CRH &= ~(0x0F << 0);    
	GPIOA->CRH |= (0x03 << 0);         // PA8 (CSN): Output push-pull 50MHz
	
	SPI2->CR1 &= ~(1 << 6);            // T?t SPI2 d? c?u hình
	SPI2->CR1 |= (3 << 3);             // B? chia 16 (8MHz / 16 = 500kHz)
	SPI2->CR1 |= (1 << 2);             // Master selection
	SPI2->CR1 |= (1 << 8) | (1 << 9);  // Software slave management (SSM & SSI)
	SPI2->CR1 &= ~(1 << 0) & ~(1 << 1); // Mode 0 (CPOL=0, CPHA=0)
	SPI2->CR1 |= (1 << 6);             // B?t SPI2

  SPI1->CR1 &= ~(1 << 6); // off to config
  SPI1->CR1 |= (3 << 3); // Fplck/64
  SPI1->CR1 |= (1 << 2); // master selection
  SPI1->CR1 |= (1 << 8)|(1 << 9);
  SPI1->CR1 &= ~(1 << 0) &~(1 << 1); // CPOL = 0, CPHA = 0
  SPI1->CR1 |= (1 << 6);

  nRF24L01_Config();
  nRF2_Config_Receiver();
  uint8_t data[16] = "MyBest";
	memset(rx_buffer,0,sizeof(rx_buffer));
	nRF2_Write_Reg(0x00, 0x0F);
	nRF2_Write_Reg(0x01, 0x00);
	nRF_Transmit(data,16); // truyền data
	HAL_Delay(5);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_Delay(5);
    status2 = nRF2_Read_Reg(0x07); 
    if (status2 & (1 << 6)) {      
        nRF2_Receive_Payload(rx_buffer, 16);
    }
    config2 = nRF2_Read_Reg(0x05); 
		HAL_Delay(100);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
