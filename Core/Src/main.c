/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "i2c.h"
#include "i2s.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "watch_config.h"	// THIS MUST BE INCLUDED FIRST

#ifdef VIBRATION
#include "morse_driver.h"
#endif
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
#ifdef DEBUG_TERMINAL
uint8_t rx_buffer[2];
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef DEBUG_TERMINAL
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
    	switch(*((uint16_t*)rx_buffer))
    	{
			#ifdef DATE_TIME
    		case 0x4954:	// Tell Time
    			HAL_UART_Transmit(&huart1, "Time\r\n", 7, 1000);
				break;
    		case 0x4144:	// Tell Date
    			HAL_UART_Transmit(&huart1, "Date\r\n", 7, 1000);
    			break;
			#endif

    		#ifdef HEART_RATE_MEASURE
    		case 0x5248:	// Heart Rate
    			HAL_UART_Transmit(&huart1, "Heart Rate\r\n", 13, 1000);
    		    break;
			#endif

			#ifdef PULSE_OX_MEASURE
    		case 0x4F50:	// Pulse Ox
    			HAL_UART_Transmit(&huart1, "Pulse Ox\r\n", 11, 1000);
    			break;
			#endif

    		case 0x5556:	// Volume Up
    			HAL_UART_Transmit(&huart1, "Volume Up\r\n", 12, 1000);
    			break;
    		case 0x4456:	// Volume Down
    			HAL_UART_Transmit(&huart1, "Volume Down\r\n", 13, 1000);
    			break;
    		case 0x5541:	// Arrow Up
    			HAL_UART_Transmit(&huart1, "Arrow Up\r\n", 11, 1000);
    			break;
    		case 0x4441:	// Arrow Down
    			HAL_UART_Transmit(&huart1, "Arrow Down\r\n", 13, 1000);
    			break;
    		case 0x4553:	// Select
    			HAL_UART_Transmit(&huart1, "Select\r\n", 9, 1000);
    			break;

    		default:
    			break;
    	}

    	memset(rx_buffer, 0, sizeof(rx_buffer));	// Clears the RX buffer
    }
}
#endif


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim == &htim21)	// VIB Control Timer
    {
		#ifdef VIBRATION
    	switch(MorseFSM_Run())
    	{
    		case DOT:
    		case DASH:
    			HAL_GPIO_WritePin(VIB_CTRL_GPIO_Port, VIB_CTRL_Pin, GPIO_PIN_SET);
    			break;

    		case PAUSE:
    		case SPACE:
    			HAL_GPIO_WritePin(VIB_CTRL_GPIO_Port, VIB_CTRL_Pin, GPIO_PIN_RESET);
    			break;

    		case DONE:
    			HAL_GPIO_WritePin(VIB_CTRL_GPIO_Port, VIB_CTRL_Pin, GPIO_PIN_RESET);
    			HAL_TIM_Base_Stop_IT(&htim21);
    			break;

    		default:
    			break;
    	}
		#endif
    }
}

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
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  MX_RTC_Init();
  MX_ADC_Init();
  MX_USART1_UART_Init();
  MX_TIM21_Init();
  /* USER CODE BEGIN 2 */
  MorseFSM_Prime(". .-- ... .--");
  HAL_TIM_Base_Start_IT(&htim21);
  //uint8_t test_data = 0x12;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //HAL_UART_Receive(&huart1, usart_buf, 256, 1000);
	  //HAL_I2C_Master_Transmit(&hi2c1, 84, (unsigned char*) "Hello World!", 12, 2000);
	  //HAL_UART_Transmit(&huart1, usart_buf, 256, 1000);
	  /*
	  #ifdef DEBUG_TERMINAL
	  HAL_UART_Receive_IT(&huart1, rx_buffer, 2);
	  #endif
	  */

	  //HAL_SPI_Transmit(&hspi1, &test_data, 1, 1000);
	  //HAL_GPIO_TogglePin(GPIOA, VIB_CTRL_Pin);
	  // HAL_Delay(1000);
    /* USER CODE END WHILE */

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.LSEState = RCC_LSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USB;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);

  /** Enable the SYSCFG APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
  */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1000);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
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
