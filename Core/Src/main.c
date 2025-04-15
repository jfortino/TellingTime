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
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "i2s.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "watch_config.h"	// THIS MUST BE INCLUDED FIRST

#ifdef USB_DRIVE
#include "tusb.h"
#endif

#ifdef VIBRATION
#include "morse_driver.h"
#endif

#ifdef SPI_FLASH
#include "W25Q128JV_driver.h"
#endif

#ifdef AUDIO
#include "audio_player.h"
#include "audio_library.h"
#endif

#ifdef BIOSIGNALS
#include "MAX30101_driver.h"
#endif
#ifdef EXTI_SIGNALS
#include "button_fsm.h"
#endif

#include "event_queue.h"

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
#ifdef EXTI_SIGNALS
const GPIO_TypeDef* EXTI_SIGNAL_PORTS[] = {GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA};
#endif

#ifdef DEBUG_TERMINAL
uint8_t rx_buffer[2];

#ifdef VIBRATION
char morse_buffer[15];
uint8_t morse_rx_flag = 0;
#endif

#endif

#ifdef AUDIO
uint16_t audio_buffers[AUDIO_BUFFER_SAMPLES * 2 * NUM_AUDIO_BUFFERS];
uint8_t fill_audio_buffer_flag = 0;
uint8_t play_audio_flag = 0;
EAudioFile audio_list[1] = {ZERO_AUDIO};
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
		#ifdef VIBRATION
    	if (morse_rx_flag)
    	{
    		morse_rx_flag = 0;
    		MorseFSM_Prime((const char*) morse_buffer);
    		HAL_TIM_Base_Start_IT(&htim21);
    	}
    	else
    	{
		#endif
			switch(*((uint16_t*)rx_buffer))
			{
				#ifdef DATE_TIME
				case 0x4954:	// Tell Time (TI)
					HAL_UART_Transmit(&huart1, "Time\r\n", 7, 1000);
					break;
				case 0x4144:	// Tell Date (DA)
					HAL_UART_Transmit(&huart1, "Date\r\n", 7, 1000);
					break;
				#endif

				#ifdef BIOSIGNALS
				case 0x5248:	// Heart Rate (HR)
					HAL_UART_Transmit(&huart1, "Heart Rate\r\n", 13, 1000);
					break;

				case 0x4F50:	// Pulse Ox (PO)
					HAL_UART_Transmit(&huart1, "Pulse Ox\r\n", 11, 1000);
					break;
				#endif

				#ifdef AUDIO
				case 0x5556:	// Volume Up (VU)
					HAL_UART_Transmit(&huart1, "Volume Up\r\n", 12, 1000);
					break;
				case 0x4456:	// Volume Down (VD)
					HAL_UART_Transmit(&huart1, "Volume Down\r\n", 13, 1000);
					break;
				case 0x4150:	// Play Audio (PA)
					HAL_UART_Transmit(&huart1, "Play Audio\r\n", 13, 1000);
					play_audio_flag = 1;
					PlayerFSM_Prime(audio_list, 1, audio_buffers);
					HAL_I2S_Transmit_DMA(&hi2s2, audio_buffers, AUDIO_BUFFER_SAMPLES * 2 * NUM_AUDIO_BUFFERS);
					break;
				#endif

				case 0x5541:	// Arrow Up (AU)
					HAL_UART_Transmit(&huart1, "Arrow Up\r\n", 11, 1000);
					break;
				case 0x4441:	// Arrow Down (AD)
					HAL_UART_Transmit(&huart1, "Arrow Down\r\n", 13, 1000);
					break;
				case 0x4553:	// Select (SE)
					HAL_UART_Transmit(&huart1, "Select\r\n", 9, 1000);
					break;

				#ifdef VIBRATION
				case 0x4F4D:	// Morse (MO)
					HAL_UART_Transmit(&huart1, "Morse\r\n", 8, 1000);
					HAL_UART_Receive_IT(&huart1, morse_buffer, 14);
					morse_rx_flag = 1;
					break;
				#endif

				default:
					break;
			}

			memset(rx_buffer, 0, sizeof(rx_buffer));	// Clears the RX buffer
		#ifdef VIBRATION
    	}
		#endif
    }
}
#endif


#ifdef AUDIO
#if NUM_AUDIO_BUFFERS == 2
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
	if (play_audio_flag)
	{
		fill_audio_buffer_flag = 1;
	}
	else
	{
		HAL_I2S_DMAStop(&hi2s2);
	}
}
#endif


void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	if (play_audio_flag)
	{
		fill_audio_buffer_flag = 1;
	}
	else
	{
		HAL_I2S_DMAStop(&hi2s2);
	}
}
#endif


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	#ifdef VIBRATION
	// Vibration motor timer
    if (htim == &htim21)
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

    		case MORSE_DONE:
    			HAL_GPIO_WritePin(VIB_CTRL_GPIO_Port, VIB_CTRL_Pin, GPIO_PIN_RESET);
    			HAL_TIM_Base_Stop_IT(&htim21);
    			break;

    		default:
    			break;
    	}

    	return;
		#endif
    }
	#endif

	#ifdef EXTI_SIGNALS
    // Button debounce timer
    if (htim == &htim22)
    {
    	HAL_TIM_Base_Stop_IT(&htim22);

    	if (Button_GetState() == INITIAL_PRESS)
    	{
			uint8_t button_exti_line = Button_GetButton();

			if (HAL_GPIO_ReadPin(EXTI_SIGNAL_PORTS[button_exti_line], (0x0001 << button_exti_line)) == GPIO_PIN_RESET)
			{
				Button_DebounceComplete();
				__HAL_TIM_CLEAR_FLAG(&htim2, TIM_SR_UIF); // Stops TIM2's interrupt from firing immediately
				HAL_TIM_Base_Start_IT(&htim2);
			}
			else
			{
				Button_Release();
			}
    	}

    	return;
    }

    // Long button press timer
    if (htim == &htim2)
    {
    	HAL_TIM_Base_Stop_IT(&htim2);

    	if (Button_GetState() == DEBOUNCED)
    	{
    		Button_LongPress();
    		EventQueue_Enqueue(Button_GetWatchEvent());
    	}

		return;
    }
	#endif
}
#endif

#ifdef EXTI_SIGNALS
uint8_t getEXTILineNum(uint16_t GPIO_Pin)
{
	for (uint8_t i = 0; i < 16; i++)
	{
		if (GPIO_Pin & (0x0001 << i))
		{
			return i;
		}
	}

	return 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint8_t exti_line_num = getEXTILineNum(GPIO_Pin);
	GPIO_PinState pin_state = HAL_GPIO_ReadPin(EXTI_SIGNAL_PORTS[exti_line_num], GPIO_Pin);

	switch (GPIO_Pin)
	{
		case SPO2_INT:
			break;

		case CHARGE_STAT_INT:
			EventQueue_Enqueue(pin_state == GPIO_PIN_RESET ? CHARGE_CYCLE_START : CHARGE_CYCLE_END);
			break;

		case LOW_BATTERY_INT:
			break;

		default:	// If the interrupt signal did not come from the above three cases, it must be a button
			switch (Button_GetState())
			{
				case BUTTON_IDLE:	// This runs when the button is initially pressed
					if (EventQueue_GetNumQueued() == 0)	// Watch functions controlled by buttons are blocked if there is already another event
					{
						Button_InitPress(exti_line_num);
						__HAL_TIM_CLEAR_FLAG(&htim22, TIM_SR_UIF); // Stops TIM22's interrupt from firing immediately
						HAL_TIM_Base_Start_IT(&htim22);
					}
					break;

				case DEBOUNCED:		// This runs once the button is debounced and released before the long press timer overflows
					Button_ShortPress();
					EventQueue_Enqueue(Button_GetWatchEvent());
					Button_Release();
					break;

				case LONG_PRESS:	// This runs once the button is debounced and released after the long press timer overflows
					Button_Release();
					break;

				default:
					break;
			}
			break;
	}
}
#endif


void handle_events()
{
	while (EventQueue_GetNumQueued() > 0)
	{
		switch (EventQueue_Peek())
		{
			case GET_TIME_MIN:
				HAL_UART_Transmit(&huart1, "Time Min\r\n", 11, 1000);
				EventQueue_Dequeue();
				break;

			case GET_TIME_FULL:
				HAL_UART_Transmit(&huart1, "Time Full\r\n", 12, 1000);
				EventQueue_Dequeue();
				break;

			case GET_DATE_DAY:
				break;

			case GET_DATE_FULL:
				break;

			case GET_HR:
				HAL_UART_Transmit(&huart1, "Heart Rate\r\n", 13, 1000);
				EventQueue_Dequeue();
				break;

			case GET_SPO2:
				break;

			case VOL_UP:
				break;

			case VOL_DOWN:
				break;

			case VOL_MUTE:
				break;

			case VOL_UNMUTE:
				break;

			case MENU_UP:
				break;

			case MENU_DOWN:
				break;

			case MENU_SELECT:
				break;

			case CHARGE_CYCLE_START:
				break;

			case CHARGE_CYCLE_END:
				break;

			case LOW_BATTERY_DETECT:
				break;

			default:
				break;
		}
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_SPI1_Init();
  MX_RTC_Init();
  MX_ADC_Init();
  MX_USART1_UART_Init();
  MX_USB_PCD_Init();
  MX_FATFS_Init();
  MX_TIM21_Init();
  /* USER CODE BEGIN 2 */

  #ifdef SPI_FLASH
  if (Flash_Init(&hspi1, FLASH_CS_GPIO_Port, FLASH_CS_Pin) != FLASH_OK)
  {
	  Error_Handler();
  }
  #endif

  #ifdef USB_DRIVE
  tusb_init();
  #endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //HAL_UART_Receive(&huart1, usart_buf, 256, 1000);
	  //HAL_I2C_Master_Transmit(&hi2c1, 84, (unsigned char*) "Hello World!", 12, 2000);
	  //HAL_UART_Transmit(&huart1, usart_buf, 256, 1000);
	  #ifdef USB_DRIVE
	  tud_task();
	  #endif

	  #ifdef DEBUG_TERMINAL
	  #ifdef VIBRATION
	  if (!morse_rx_flag)
	  {
	  #endif
	  	  HAL_UART_Receive_IT(&huart1, rx_buffer, 2);
	  #ifdef VIBRATION
	  }
	  #endif


	  #ifdef AUDIO
	  if (fill_audio_buffer_flag)
	  {
		  if (PlayerFSM_Run() != AUDIOPLAYER_OK)
		  {
			  play_audio_flag = 0;
		  }

		  fill_audio_buffer_flag = 0;

		  #if NUM_AUDIO_BUFFERS == 1
		  HAL_I2S_Transmit_DMA(&hi2s2, audio_buffers, AUDIO_BUFFER_SAMPLES * 2);
		  #endif
	  }
 	  #endif
	  #endif /*DEBUG_TERMINAL*/

	  handle_events();


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.LSEState = RCC_LSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USB;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

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
