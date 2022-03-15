/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "peripheral_init.h"
#include "amt_inc_enc.h"
#include "utilities.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPI_TX_BUF_SIZE 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
inc_enc_t encoder[2];

uint8_t spi_tx_buf[SPI_TX_BUF_SIZE] = {
	0
};

uint8_t spi_rx_buf[SPI_TX_BUF_SIZE] = {
	0
};

float prev_vel[2] = {
	0
};

#define EXPONENTIAL_ALPHA 0.8

uint32_t diff_t = 0;
uint32_t cmplt_t = 0;
uint8_t tx_rx_state = 0; //0 incomplete, 1 complete

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
    //Configure systick_callback rate and registration
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / FREQUENCY);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    Peripheral_Init();
    /* USER CODE BEGIN 2 */
    AMT_Inc_Init(&encoder[LEFT_INDEX], &htim2);
    AMT_Inc_Init(&encoder[RIGHT_INDEX], &htim3);

    //Pull CS Low to Init Transmission
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_DMA(&hspi1, spi_tx_buf, spi_rx_buf, sizeof(spi_tx_buf));
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
	__NOP();

    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {
	    0
    };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {
	    0
    };

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
	Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
	Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
/**
 * @brief  SYSTICK callback.
 * @retval None
 */
void HAL_SYSTICK_Callback(void) {
    /* NOTE : This function Should not be modified, when the callback is needed,
     the HAL_SYSTICK_Callback could be implemented in the user file
     */
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    encoder[LEFT_INDEX].enc = AMT_ReadEncoder(&encoder[LEFT_INDEX], (int16_t) (LEFT_ENC_TIM->CNT));
    AMT_CalVelocity(&encoder[LEFT_INDEX]);
    encoder[RIGHT_INDEX].enc = AMT_ReadEncoder(&encoder[RIGHT_INDEX], (int16_t) (RIGHT_ENC_TIM->CNT));
    AMT_CalVelocity(&encoder[RIGHT_INDEX]);
    //Exponential filter for each velocity
//    encoder[RIGHT_INDEX].velocity = encoder[RIGHT_INDEX].velocity * EXPONENTIAL_ALPHA + (1.0 - EXPONENTIAL_ALPHA) * prev_vel[RIGHT_INDEX];
//    encoder[LEFT_INDEX].velocity = encoder[LEFT_INDEX].velocity * EXPONENTIAL_ALPHA + (1.0 - EXPONENTIAL_ALPHA) * prev_vel[LEFT_INDEX];
//    encoder[RIGHT_INDEX].velocity *= 0.25;
//    encoder[LEFT_INDEX].velocity *= 0.25;
//    prev_vel[LEFT_INDEX] = encoder[LEFT_INDEX].velocity;
//    prev_vel[RIGHT_INDEX] = encoder[RIGHT_INDEX].velocity;

//SPI Transmit
    floatuint8_t tx_buf[2];
    tx_buf[LEFT_INDEX].b32 = encoder[LEFT_INDEX].velocity;
    tx_buf[RIGHT_INDEX].b32 = encoder[RIGHT_INDEX].velocity;
    spi_tx_buf[0] = tx_buf[LEFT_INDEX].b8[0];
    spi_tx_buf[1] = tx_buf[LEFT_INDEX].b8[1];
    spi_tx_buf[2] = tx_buf[LEFT_INDEX].b8[2];
    spi_tx_buf[3] = tx_buf[LEFT_INDEX].b8[3];
    spi_tx_buf[4] = tx_buf[RIGHT_INDEX].b8[0];
    spi_tx_buf[5] = tx_buf[RIGHT_INDEX].b8[1];
    spi_tx_buf[6] = tx_buf[RIGHT_INDEX].b8[2];
    spi_tx_buf[7] = tx_buf[RIGHT_INDEX].b8[3];
    uint16_t checksum = CalculateChecksum_16bit(spi_tx_buf, 8);
    spi_tx_buf[8] = (uint8_t) (checksum & 0xff);
    spi_tx_buf[9] = (uint8_t) ((checksum >> 8) & 0xff);

    if (tx_rx_state == 1 || (tx_rx_state == 0 && HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_READY)) {
	//Pull CS Low to Init Transmission
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_DMA(&hspi1, spi_tx_buf, spi_rx_buf, sizeof(spi_tx_buf));
	tx_rx_state = 0;
	diff_t = HAL_GetTick() - cmplt_t;
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    /* NOTE : This function should not be modified, when the callback is needed,
     the HAL_SPI_TxRxCpltCallback should be implemented in the user file
     */
    if (hspi == &hspi1) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	tx_rx_state = 1;

	if(ValidateChecksum_16bit(spi_rx_buf, sizeof(spi_rx_buf)) == true)
	{
	    cmplt_t = HAL_GetTick();
	}
    }
}

//void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
//    /* NOTE : This function should not be modified, when the callback is needed,
//     the HAL_SPI_TxCpltCallback should be implemented in the user file
//     */
//    if (hspi == &hspi1) {
//
////	HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*) spi_tx_buf, sizeof(spi_tx_buf));
//    }
//
//}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
