/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uart_bitbanging.h"
#include "i2c_bitbanging.h"
#include "24c256_bitbanging.h"
#include <string.h>
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
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char msg[] = "Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum";
uint8_t rx_string[64];

volatile char uart_rx_buffer[UART_RX_BUFFER_SIZE];
volatile uint16_t uart_rx_buffer_index = 0;
volatile uint8_t string_ready_flag = 0;
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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  DWT_Delay_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  UART_Receive_String_Blocking();
	  process_UART_command();
    /* USER CODE BEGIN 3 */


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	  // Reset RCC to default state
	  RCC->CR |= RCC_CR_HSION;
	  RCC->CFGR = 0x00000000;
	  RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_PLLON);
	  RCC->CR &= ~RCC_CR_HSEBYP;
	  RCC->CIR = 0x00000000;

	  // Enable Power Interface clock
	  RCC->APB1ENR |= RCC_APB1ENR_PWREN;

	  // Configure voltage scaling
	  PWR->CR |= PWR_CR_VOS;

	  // Enable HSE and wait for it to be ready
	  RCC->CR |= RCC_CR_HSEBYP | RCC_CR_HSEON;
	  while(!(RCC->CR & RCC_CR_HSERDY));

	  // Configure Flash prefetch and latency
	  FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;

	  // Configure PLL
	  RCC->PLLCFGR = (4 << RCC_PLLCFGR_PLLM_Pos) |    // PLLM = 4
	                 (168 << RCC_PLLCFGR_PLLN_Pos) |   // PLLN = 168
	                 (0 << RCC_PLLCFGR_PLLP_Pos) |     // PLLP = 2 (0 means divide by 2)
	                 (7 << RCC_PLLCFGR_PLLQ_Pos) |     // PLLQ = 7
	                 RCC_PLLCFGR_PLLSRC_HSE;           // HSE as PLL source

	  // Enable PLL and wait for it to be ready
	  RCC->CR |= RCC_CR_PLLON;
	  while(!(RCC->CR & RCC_CR_PLLRDY));

	  // Configure clock dividers
	  RCC->CFGR = (0 << RCC_CFGR_HPRE_Pos) |    // AHB prescaler = 1
	              (4 << RCC_CFGR_PPRE1_Pos) |    // APB1 prescaler = 4
	              (2 << RCC_CFGR_PPRE2_Pos);     // APB2 prescaler = 2

	  // Switch to PLL as system clock
	  RCC->CFGR |= RCC_CFGR_SW_PLL;
	  while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

	  // Update SystemCoreClock variable
	  SystemCoreClock = 168000000;
}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    // Enable GPIO Ports Clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // Enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;  // Enable GPIOB clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;  // Enable GPIOH clock

    // Configure PA2 (Output Push Pull)
    GPIOA->MODER &= ~(3 << (2 * 2));      // Clear mode bits
    GPIOA->MODER |= (1 << (2 * 2));       // Set as output
    GPIOA->OTYPER &= ~(1 << 2);           // Push-pull
    GPIOA->OSPEEDR &= ~(3 << (2 * 2));    // Clear speed bits
    GPIOA->OSPEEDR |= (2 << (2 * 2));     // High speed
    GPIOA->PUPDR &= ~(3 << (2 * 2));      // No pull-up/pull-down
    GPIOA->BSRR = (1 << 2);               // Set PA2 high

    // Configure PA3 (Input)
    GPIOA->MODER &= ~(3 << (3 * 2));      // Clear mode bits
    GPIOA->PUPDR &= ~(3 << (3 * 2));      // No pull-up/pull-down

    // Configure PB10 and PB11 (Open Drain Output)
    GPIOB->MODER &= ~(3 << (10 * 2));     // Clear mode bits for PB10
    GPIOB->MODER |= (1 << (10 * 2));      // Set PB10 as output
    GPIOB->MODER &= ~(3 << (11 * 2));     // Clear mode bits for PB11
    GPIOB->MODER |= (1 << (11 * 2));      // Set PB11 as output
    GPIOB->OTYPER |= (1 << 10) | (1 << 11); // Open-drain
    GPIOB->OSPEEDR &= ~(3 << (10 * 2));   // Clear speed bits
    GPIOB->OSPEEDR |= (2 << (10 * 2));    // High speed
    GPIOB->OSPEEDR &= ~(3 << (11 * 2));   // Clear speed bits
    GPIOB->OSPEEDR |= (2 << (11 * 2));    // High speed
    GPIOB->PUPDR &= ~(3 << (10 * 2));     // No pull-up/pull-down
    GPIOB->PUPDR &= ~(3 << (11 * 2));     // No pull-up/pull-down
    GPIOB->BSRR = (1 << 10) | (1 << 11);  // Set PB10 and PB11 low
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
