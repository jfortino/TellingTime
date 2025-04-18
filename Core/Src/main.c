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
struct WatchSettings
{
	uint8_t volume_level;
	EFeedbackMode feedback_mode;
	EDateFormat date_format;
	ETimeFormat time_format;
	/*
	uint8_t morse_dot_length;
	uint8_t morse_dash_length;
	uint8_t morse_space_length
	*/
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
struct WatchSettings watch_settings = {0, AUDIO_MODE, DATE_MDY, TIME_TWELVE_HOUR};

#ifdef EXTI_SIGNALS
const GPIO_TypeDef* EXTI_SIGNAL_PORTS[16] = {GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOB, GPIOA, GPIOA, GPIOB, GPIOB, GPIOB, GPIOB, GPIOA, GPIOA, GPIOA, GPIOA};
#endif

#ifdef DEBUG_TERMINAL
uint8_t rx_buffer[2];
#ifdef VIBRATION
uint8_t morse_rx_flag = 0;
#endif
#endif /*DEBUG_TERMINAL*/

#ifdef VIBRATION
char morse_buffer[MORSE_BUFFER_SIZE] = ".-- - ...";
uint8_t play_morse_flag = NOT_STARTED;
#endif

#ifdef AUDIO
uint16_t audio_buffers[AUDIO_BUFFER_SAMPLES * 2 * NUM_AUDIO_BUFFERS];
uint8_t fill_audio_buffer_flag = 0;
EEventFlag play_audio_flag = NOT_STARTED;
EAudioFile audio_list[MAX_AUDIO_LIST_SIZE] = {FOUR_AUDIO};
uint8_t audio_list_size = 1;
#endif

#ifdef BIOSIGNALS
uint32_t num_bio_samples;
EEventFlag spo2_measurement_flag = NOT_STARTED;
uint8_t process_spo2_data_flag = 0;
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
				EventQueue_Enqueue(GET_HR);
				break;

			case 0x4F50:	// Pulse Ox (PO)
				HAL_UART_Transmit(&huart1, "Pulse Ox\r\n", 11, 1000);
				EventQueue_Enqueue(GET_PO);
				break;
			#endif

			case 0x5556:	// Volume Up (VU)
				HAL_UART_Transmit(&huart1, "Volume Up\r\n", 12, 1000);
				EventQueue_Enqueue(VOL_UP);
				break;
			case 0x4456:	// Volume Down (VD)
				HAL_UART_Transmit(&huart1, "Volume Down\r\n", 14, 1000);
				EventQueue_Enqueue(VOL_DOWN);
				break;
			case 0x554D:
				HAL_UART_Transmit(&huart1, "Volume Mute\r\n", 14, 1000);
				EventQueue_Enqueue(VOL_MUTE);
				break;
			case 0x4D55:
				HAL_UART_Transmit(&huart1, "Volume Unmute\r\n", 16, 1000);
				EventQueue_Enqueue(VOL_UNMUTE);
				break;

			#ifdef AUDIO
			case 0x4150:	// Play Audio (PA)
				HAL_UART_Transmit(&huart1, "Play Audio\r\n", 13, 1000);
				play_audio_flag = RUNNING;
				PlayerFSM_Prime(audio_list, 1, audio_buffers);
				HAL_I2S_Transmit_DMA(&hi2s2, audio_buffers, AUDIO_BUFFER_SAMPLES * 2 * NUM_AUDIO_BUFFERS);
				break;
			#endif

			case 0x5541:	// Arrow Up (AU)
				HAL_UART_Transmit(&huart1, "Arrow Up\r\n", 11, 1000);
				HAL_GPIO_WritePin(GPIOB, FLASH_RST_Pin, GPIO_PIN_SET);
				break;
			case 0x4441:	// Arrow Down (AD)
				HAL_UART_Transmit(&huart1, "Arrow Down\r\n", 13, 1000);
				HAL_GPIO_WritePin(GPIOB, FLASH_RST_Pin, GPIO_PIN_RESET);
				break;
			case 0x4553:	// Select (SE)
				HAL_UART_Transmit(&huart1, "Select\r\n", 9, 1000);
				break;

			#ifdef VIBRATION
			case 0x4F4D:	// Morse (MO)
				HAL_UART_Transmit(&huart1, "Morse\r\n", 8, 1000);
				HAL_UART_Receive_IT(&huart1, morse_buffer, MORSE_BUFFER_SIZE);
				break;
			#endif

			default:
				break;
		}

		memset(rx_buffer, 0, sizeof(rx_buffer));	// Clears the RX buffer
    }
}
#endif


#ifdef AUDIO
#if NUM_AUDIO_BUFFERS == 2
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
	if (play_audio_flag == RUNNING)
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
	if (play_audio_flag == RUNNING)
	{
		fill_audio_buffer_flag = 1;
	}
	else
	{
		HAL_I2S_DMAStop(&hi2s2);
	}
}


void fill_audio_buffer()
{
	  if (PlayerFSM_Run() != AUDIOPLAYER_OK)
	  {
		  play_audio_flag = FINISHED;
		  Flash_PowerDown();
	  }

	  fill_audio_buffer_flag = 0;

	  #if NUM_AUDIO_BUFFERS == 1
	  HAL_I2S_Transmit_DMA(&hi2s2, audio_buffers, AUDIO_BUFFER_SAMPLES * 2);
	  #endif
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
    			HAL_GPIO_WritePin(VIB_MOTOR_GPIO_Port, VIB_MOTOR_Pin, GPIO_PIN_SET);
    			break;

    		case PAUSE:
    		case SPACE:
    			HAL_GPIO_WritePin(VIB_MOTOR_GPIO_Port, VIB_MOTOR_Pin, GPIO_PIN_RESET);
    			break;

    		case MORSE_DONE:
    			HAL_GPIO_WritePin(VIB_MOTOR_GPIO_Port, VIB_MOTOR_Pin, GPIO_PIN_RESET);
    			HAL_TIM_Base_Stop_IT(&htim21);
    			play_morse_flag = FINISHED;
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
	//GPIO_PinState pin_state = HAL_GPIO_ReadPin(EXTI_SIGNAL_PORTS[exti_line_num], GPIO_Pin);

	switch (exti_line_num)
	{
		case SPO2_INT:			// This pin is only set up for falling edge detection
			#ifdef BIOSIGNALS
			process_spo2_data_flag = 1;
			#endif
			break;

		case MCP_STAT_INT:
			EventQueue_Enqueue(HAL_GPIO_ReadPin(EXTI_SIGNAL_PORTS[exti_line_num], GPIO_Pin) == GPIO_PIN_RESET ? CHARGE_CYCLE_START : CHARGE_CYCLE_END);
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


#ifdef BIOSIGNALS
void process_spo2_data()
{
	uint8_t fired_ints[5];
	uint8_t num_fired_ints;
	SPO2_ReadInterrupts(fired_ints, &num_fired_ints);

	for (int i = 0; i < num_fired_ints; i++)
	{
		switch (fired_ints[i])
		{
			case PWR_RDY_MASK:
				break;

			case ALC_OVF_MASK:
				break;

			case PPG_RDY_MASK:
				uint32_t new_sample[4];
				SPO2_ReadNewSample(new_sample);

				num_bio_samples--;
				if (num_bio_samples == 0)
				{
					SPO2_StopMeasurement();
					spo2_measurement_flag = FINISHED;
				}


				char sample_str[10];
				memset(sample_str, 0, 10);
				sprintf(sample_str, "%lu\r\n", new_sample[0]);
				HAL_UART_Transmit(&huart1, sample_str, 10, 1000);
				/*Do spo2 data processing here*/
				break;

			case A_FULL_MASK:
				break;

			case DIE_TEMP_RDY_MASK:
				break;

			default:
				break;
		}
	}

	process_spo2_data_flag = 0;
}
#endif



void set_gain_pins()
{
	HAL_GPIO_WritePin(GAIN_3DB_GPIO_Port, GAIN_3DB_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GAIN_9DB_GPIO_Port, GAIN_9DB_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GAIN_12DB_GPIO_Port, GAIN_12DB_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GAIN_15DB_GPIO_Port, GAIN_15DB_Pin, GPIO_PIN_RESET);

	switch (watch_settings.volume_level)
	{
		case 0:
			HAL_GPIO_WritePin(GAIN_3DB_GPIO_Port, GAIN_3DB_Pin, GPIO_PIN_SET);
			break;

		case 1:
			HAL_GPIO_WritePin(GAIN_9DB_GPIO_Port, GAIN_9DB_Pin, GPIO_PIN_SET);
			break;

		case 2:
			HAL_GPIO_WritePin(GAIN_12DB_GPIO_Port, GAIN_12DB_Pin, GPIO_PIN_SET);
			break;

		case 3:
			HAL_GPIO_WritePin(GAIN_15DB_GPIO_Port, GAIN_15DB_Pin, GPIO_PIN_SET);
			break;

		default:
			break;
	}
}


#ifdef DATE_TIME
uint8_t convert_24hr_to_12hr(uint8_t hour_24)
{
	uint8_t hour_12 = hour_24 % 12;

	if (hour_12 == 0)
	{
		hour_12 = 12;
	}

	if (hour_24 / 12 == 1)
	{
		hour_12 |= 0x80;	//AM/PM information is encoded in the MSB: AM = 0, PM = 1;
	}

	return hour_12;
}
#endif


void start_user_feedback()
{
	if (watch_settings.feedback_mode == AUDIO_MODE)
	{
		Flash_ReleasePowerDown();	// Wakes flash memory up from power down mode
		play_audio_flag = RUNNING;
		PlayerFSM_Prime(audio_list, 1, audio_buffers);	// Need to add error handling to this
		HAL_I2S_Transmit_DMA(&hi2s2, audio_buffers, AUDIO_BUFFER_SAMPLES * 2 * NUM_AUDIO_BUFFERS);
	}
	else if (watch_settings.feedback_mode == VIBRATION_MODE)
	{
		play_morse_flag = RUNNING;
		MorseFSM_Prime(morse_buffer);
		HAL_TIM_Base_Start_IT(&htim21);
	}
}


void handle_events()	// Must be called repeatedly in main loop
{
	switch (EventQueue_Peek())
	{
		#ifdef DATE_TIME
		case GET_TIME_MIN:
			switch (play_audio_flag)
			{
			case NOT_STARTED:
				//play_audio_flag = RUNNING;
				//PlayerFSM_Prime(audio_list, 1, audio_buffers);
				//HAL_I2S_Transmit_DMA(&hi2s2, audio_buffers, AUDIO_BUFFER_SAMPLES * 2 * NUM_AUDIO_BUFFERS);
				watch_settings.feedback_mode = AUDIO_MODE;
				start_user_feedback();
				break;

			case FINISHED:
				EventQueue_Dequeue();
				play_audio_flag = NOT_STARTED;
				break;

			default:
				break;
			}
			break;

		case GET_TIME_FULL:
			HAL_UART_Transmit(&huart1, "Time Full\r\n", 12, 1000);
			EventQueue_Dequeue();
			break;

		case GET_DATE_DAY:
			switch (play_morse_flag)
			{
			case NOT_STARTED:
				watch_settings.feedback_mode = VIBRATION_MODE;
				start_user_feedback();
				break;

			case FINISHED:
				EventQueue_Dequeue();
				play_morse_flag = NOT_STARTED;
				break;

			default:
				break;
			}
			break;

		case GET_DATE_FULL:
			HAL_UART_Transmit(&huart1, "Date Full\r\n", 12, 1000);
			EventQueue_Dequeue();
			break;
		#endif

		#ifdef BIOSIGNALS
		case GET_HR:
			switch (spo2_measurement_flag)
			{
				case NOT_STARTED:
					num_bio_samples = HR_SAMPLE_NUM;
					SPO2_StartMeasurement(MEASURE_HR);
					spo2_measurement_flag = RUNNING;
					break;

				case FINISHED:
					// Send results to audio player
					EventQueue_Dequeue();
					spo2_measurement_flag = NOT_STARTED;
					break;

				default:
					break;
			}
			break;

		case GET_PO:
			switch (spo2_measurement_flag)
			{
				case NOT_STARTED:
					num_bio_samples = SPO2_SAMPLE_NUM;
					SPO2_StartMeasurement(MEASURE_SPO2);
					spo2_measurement_flag = RUNNING;
					break;

				case FINISHED:
					// Send results to audio player
					EventQueue_Dequeue();
					spo2_measurement_flag = NOT_STARTED;
					break;

				default:
					break;
			}
			break;
		#endif

		case VOL_UP:
			if (watch_settings.feedback_mode == AUDIO_MODE && watch_settings.volume_level < 3)
			{
				watch_settings.volume_level++;
				set_gain_pins();
			}
			EventQueue_Dequeue();
			break;

		case VOL_DOWN:
			if (watch_settings.feedback_mode == AUDIO_MODE && watch_settings.volume_level > 0)
			{
				watch_settings.volume_level--;
				set_gain_pins();
			}
			EventQueue_Dequeue();
			break;

		case VOL_MUTE:
			watch_settings.feedback_mode = VIBRATION_MODE;
			EventQueue_Dequeue();
			break;

		case VOL_UNMUTE:
			watch_settings.feedback_mode = AUDIO_MODE;
			EventQueue_Dequeue();
			break;

		case MENU_UP:
			break;

		case MENU_DOWN:
			break;

		case MENU_SELECT:
			break;

		case CHARGE_CYCLE_START:
			HAL_UART_Transmit(&huart1, "Charge Start\r\n", 15, 1000);
			EventQueue_Dequeue();
			break;

		case CHARGE_CYCLE_END:
			HAL_UART_Transmit(&huart1, "Charge End\r\n", 13, 1000);
			EventQueue_Dequeue();
			break;

		case LOW_BATTERY_DETECT:
			HAL_UART_Transmit(&huart1, "Low Battery\r\n", 14, 1000);
			EventQueue_Dequeue();
			break;

		default:
			break;
	}
}


#ifndef USB_DRIVE
void MX_USB_PCD_Init() {}	// Dummy instance of this function to keep the compiler happy
#endif


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
  MX_TIM22_Init();
  MX_TIM2_Init();
  MX_TIM21_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);

  #ifdef SPI_FLASH
  if (Flash_Init(&hspi1, FLASH_CS_GPIO_Port, FLASH_CS_Pin) != FLASH_OK)
  {
	  Error_Handler();
  }
  #endif

  #ifdef BIOSIGNALS
  if (SPO2_Init(&hi2c1) != SPO2_OK)
  {
	  Error_Handler();
  }
  #endif

  set_gain_pins();

  #ifdef USB_DRIVE
  tusb_init();
  #endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
	  #endif /*DEBUG_TERMINAL*/


	  #ifdef AUDIO
	  if (fill_audio_buffer_flag)
	  {
		  fill_audio_buffer();
	  }
 	  #endif


	  #ifdef BIOSIGNALS
	  if (process_spo2_data_flag)
	  {
		  process_spo2_data();
	  }
	  #endif

	  HAL_ADC_Start(&hadc);
	  HAL_ADC_PollForConversion(&hadc, 1);
	  uint16_t adc_value = HAL_ADC_GetValue(&hadc);

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_3;
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
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
