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

#if defined(HEARTRATE) || defined(PULSE_OX)
#include "MAX30101_driver.h"
#include "arm_math.h"
#endif

#ifdef EXTI_SIGNALS
#include "button_fsm.h"
#endif

#include "event_queue.h"

#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifndef USB_DRIVE
struct WatchSettings
{
	uint8_t volume_level;
	EFeedbackMode feedback_mode;
	EDateFormat date_format;
	ETimeFormat time_format;
	uint8_t vibration_length;
	/*
	uint8_t morse_dot_length;
	uint8_t morse_dash_length;
	uint8_t morse_space_length
	*/
};
#endif
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#ifndef USB_DRIVE
struct WatchSettings watch_settings = {2, AUDIO_MODE, DATE_MDY, TIME_TWELVE_HOUR, 1};
EMenuFSMState menu_state = MENU_IDLE;
#endif

#ifdef LOW_POWER_MODE
volatile uint8_t pending_event_flag = 0;
#endif	/*LOW_POWER_MODE*/

#ifdef EXTI_SIGNALS
const GPIO_TypeDef* EXTI_SIGNAL_PORTS[16] = {GPIOA, GPIOA, GPIOB, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOB, GPIOA, GPIOB, GPIOA, GPIOC, GPIOB, GPIOC};
#endif

#ifdef DEBUG_TERMINAL
uint8_t rx_buffer[2];
#ifdef VIBRATION
uint8_t morse_rx_flag = 0;
#endif
#endif /*DEBUG_TERMINAL*/

#ifdef MEASURE_BATT
volatile uint8_t alarm_hour_inc = 0;
volatile uint8_t alarm_min_inc = 0;
volatile uint8_t alarm_sec_inc = 0;
volatile uint8_t batt_sample_num = 0;
volatile uint32_t batt_sample_sum = 0;
#endif

#if defined(VIBRATION) || defined(AUDIO)
volatile EEventFlag user_feedback_flag = NOT_STARTED;
#endif

#ifdef VIBRATION
char morse_buffer[MAX_MORSE_BUFFER_SIZE] = ".-- - ...";
#endif

#ifdef AUDIO
//uint16_t audio_buffers[AUDIO_BUFFER_SAMPLES * 2 * NUM_AUDIO_BUFFERS];
uint16_t* audio_buffers;
volatile uint8_t fill_audio_buffer_flag = 0;
EAudioFile audio_list[MAX_AUDIO_LIST_SIZE];// = {ONE_AUDIO, TWO_AUDIO, THREE_AUDIO, FOUR_AUDIO, FIVE_AUDIO, SIX_AUDIO, SEVEN_AUDIO, EIGHT_AUDIO, NINE_AUDIO, TEN_AUDIO};
uint8_t audio_list_size = 0;
#endif

#if defined(HEARTRATE) || defined(PULSE_OX)
uint16_t num_bio_samples;
EEventFlag bio_measurement_flag = NOT_STARTED;
ESPO2MeasurementType bio_measurement_type = MEASURE_HR;
uint8_t process_bio_data_flag = 0;
#endif

#ifdef HEARTRATE
q15_t* heartrate_data;
q15_t fir_state[FIR_FILTER_TAPS];
const q15_t FIR_COEFFICIENTS[FIR_FILTER_TAPS] = {-896, -673, -640, -432, -174, -43, -143, -408, -625, -601, -341, -77, -80, -409, -811, -916, -579, -66, 135, -254, -978, -1396, -1023, -52, 683, 344, -1091, -2553, -2496, -44, 4165, 8157, 9793, 8157, 4165, -44, -2496, -2553, -1091, 344, 683, -52, -1023, -1396, -978, -254, 135, -66, -579, -916, -811, -409, -80, -77, -341, -601, -625, -408, -143, -43, -174, -432, -640, -673, -896, 0};
arm_fir_instance_q15 S;
uint8_t heartrate_bpm;
#endif

#ifdef PULSE_OX
q15_t* spo2_green_data;
q15_t* spo2_ir_data;

uint8_t spo2_percent;
#endif



//arm_biquad_casd_df1_inst_q15

#ifdef DATE_TIME
RTC_TimeTypeDef set_time;
RTC_DateTypeDef set_date;
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
				EventQueue_Enqueue(GET_TIME_FULL);
				break;
			case 0x4144:	// Tell Date (DA)
				HAL_UART_Transmit(&huart1, "Date\r\n", 7, 1000);
				EventQueue_Enqueue(GET_DATE_FULL);
				break;
			#endif

			#ifdef HEARTRATE
			case 0x5248:	// Heart Rate (HR)
				HAL_UART_Transmit(&huart1, "Heart Rate\r\n", 13, 1000);
				EventQueue_Enqueue(GET_HR);
				break;
			#endif

			#ifdef PULSE_OX
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
			case 0x554D:	// Volume Mute (MU)
				HAL_UART_Transmit(&huart1, "Volume Mute\r\n", 14, 1000);
				EventQueue_Enqueue(VOL_MUTE);
				break;
			case 0x4D55:	// Volume Unmute (UM)
				HAL_UART_Transmit(&huart1, "Volume Unmute\r\n", 16, 1000);
				EventQueue_Enqueue(VOL_UNMUTE);
				break;

			#ifdef AUDIO
			case 0x4150:	// Play Audio (PA)
				HAL_UART_Transmit(&huart1, "Play Audio\r\n", 13, 1000);
				user_feedback_flag = RUNNING;
				PlayerFSM_Prime(audio_list, audio_list_size, audio_buffers);
				HAL_I2S_Transmit_DMA(&hi2s2, audio_buffers, AUDIO_BUFFER_SAMPLES * 2 * NUM_AUDIO_BUFFERS);
				break;
			#endif

			case 0x5541:	// Arrow Up (AU)
				HAL_UART_Transmit(&huart1, "Arrow Up\r\n", 11, 1000);
				EventQueue_Enqueue(MENU_UP);
				break;
			case 0x4441:	// Arrow Down (AD)
				HAL_UART_Transmit(&huart1, "Arrow Down\r\n", 13, 1000);
				EventQueue_Enqueue(MENU_DOWN);
				break;
			case 0x4553:	// Select (SE)
				HAL_UART_Transmit(&huart1, "Select\r\n", 9, 1000);
				EventQueue_Enqueue(MENU_SELECT);
				break;

			#ifdef VIBRATION
			case 0x4F4D:	// Morse (MO)
				HAL_UART_Transmit(&huart1, "Morse\r\n", 8, 1000);
				HAL_UART_Receive_IT(&huart1, morse_buffer, MAX_MORSE_BUFFER_SIZE);
				break;
			#endif

			case 0x5352:	// Reset (RS)
				HAL_UART_Transmit(&huart1, "Reset\r\n", 8, 1000);

				#ifdef SPI_FLASH
				Flash_ReleasePowerDown();
				Flash_ResetDevice();
				#endif

				#ifdef defined(HEARTRATE) || defined(PULSE_OX)
				SPO2_ResetDevice();
				#endif

				NVIC_SystemReset();
				break;

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
	if (user_feedback_flag == RUNNING)
	{
		fill_audio_buffer_flag = 1;
	}
	else
	{
		HAL_I2S_DMAStop(&hi2s2);
	}
}
#endif	/*NUM_AUDIO_BUFFERS == 2*/


void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	if (user_feedback_flag == RUNNING)
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
		  user_feedback_flag = FINISHED;
		  Flash_PowerDown();
	  }

	  fill_audio_buffer_flag = 0;

	  #if NUM_AUDIO_BUFFERS == 1
	  HAL_I2S_Transmit_DMA(&hi2s2, audio_buffers, AUDIO_BUFFER_SAMPLES * 2);
	  #endif
}
#endif	/*AUDIO*/


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
    			user_feedback_flag = FINISHED;
    			break;

    		default:
    			break;
    	}

    	return;
		#endif
    }
	#endif	/*VIBRATION*/

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
				#ifdef LOW_POWER_MODE
				pending_event_flag = 0;
				#endif	/*LOW_POWER_MODE*/
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
	#endif	/*EXTI_SIGNALS*/
}


#ifdef MEASURE_BATT
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	#ifdef LOW_POWER_MODE
	pending_event_flag = 1;		// Prevents us from going to sleep since we expect there will be an event after taking an ADC measurement
	#endif	/*LOW_POWER_MODE*/
	HAL_GPIO_WritePin(VBATT_CTRL_GPIO_Port, VBATT_CTRL_Pin, GPIO_PIN_SET);	// Biases the measurement voltage divider MOSFET
	HAL_ADC_Start_IT(&hadc);
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	// Updates the alarm so that it happens again
	RTC_AlarmTypeDef alarm;

	if (HAL_RTC_GetAlarm(&hrtc, &alarm, RTC_ALARM_A, RTC_FORMAT_BIN) != HAL_OK)
	{
		Error_Handler();
	}

	// add logic so everything below this only runs after a certain number of samples has been taken
	batt_sample_sum += HAL_ADC_GetValue(hadc);
	batt_sample_num++;

	if (batt_sample_num >= BATT_MEASURE_SAMPLES)
	{
		HAL_GPIO_WritePin(VBATT_CTRL_GPIO_Port, VBATT_CTRL_Pin, GPIO_PIN_RESET);	// Unbiases the measurement voltage divider MOSFET
		HAL_ADC_Stop_IT(hadc);		// Stops taking ADC measurements

		if (batt_sample_sum / batt_sample_num <= 1912)
		{
			EventQueue_Enqueue(LOW_BATT_DETECT);
		}

		batt_sample_sum = 0;
		batt_sample_num = 0;

		alarm.AlarmTime.Hours = (alarm.AlarmTime.Hours + alarm_hour_inc) % 24;
		alarm.AlarmTime.Minutes = (alarm.AlarmTime.Minutes + alarm_min_inc) % 60;
		alarm.AlarmTime.Seconds = (alarm.AlarmTime.Seconds + alarm_sec_inc) % 60;
		alarm.AlarmTime.SubSeconds = 0;

		if (HAL_RTC_SetAlarm_IT(&hrtc, &alarm, RTC_FORMAT_BIN) != HAL_OK)
		{
			Error_Handler();
		}

		#ifdef LOW_POWER_MODE
		pending_event_flag = 0;
		#endif	/*LOW_POWER_MODE*/
	}
}
#endif	/*MEASURE_BATT*/


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
			#if defined(HEARTRATE) || defined(PULSE_OX)
			process_bio_data_flag = 1;
			#endif
			break;

		case MCP_STAT_INT:
			#ifdef CHARGING_MONITOR
			EventQueue_Enqueue(HAL_GPIO_ReadPin(EXTI_SIGNAL_PORTS[exti_line_num], GPIO_Pin) == GPIO_PIN_RESET ? CHARGE_CYCLE_START : CHARGE_CYCLE_END);
			#endif
			break;

		default:	// If the interrupt signal did not come from the above three cases, it must be a button
			switch (Button_GetState())
			{
				case BUTTON_IDLE:	// This runs when the button is initially pressed
					if (EventQueue_GetNumQueued() == 0)	// Watch functions controlled by buttons are blocked if there is already another event
					{
						#ifdef LOW_POWER_MODE
						pending_event_flag = 1;						// Prevents us from going to sleep since we expect there will be an event after debouncing
						#endif	/*LOW_POWER_MODE*/
						Button_InitPress(exti_line_num);
						__HAL_TIM_CLEAR_FLAG(&htim22, TIM_SR_UIF); 	// Stops TIM22's interrupt from firing immediately
						HAL_TIM_Base_Start_IT(&htim22);
					}
					break;

				case DEBOUNCED:		// This runs once the button is debounced and released before the long press timer overflows
					Button_ShortPress();
					EventQueue_Enqueue(Button_GetWatchEvent());
					Button_Release();
					#ifdef LOW_POWER_MODE
					pending_event_flag = 0;
					#endif	/*LOW_POWER_MODE*/
					break;

				case LONG_PRESS:	// This runs once the button is debounced and released after the long press timer overflows
					Button_Release();
					#ifdef LOW_POWER_MODE
					pending_event_flag = 0;
					#endif	/*LOW_POWER_MODE*/
					break;

				default:
					break;
			}
			break;
	}
}
#endif	/*EXTI_SIGNALS*/

/*
uint32_t run_iir_filter(uint32_t x_input)
{
	uint32_t y_output = x_input * b[0];

	for (int i = 1; i < IIR_FILTER_STAGES; i++)
	{
		y_output += b[i] * iir_delayed_x[i-1];
		y_output += a[i] * iir_delayed_y[i-1];

		iir_delayed_x[i] = iir_delayed_x[i-1];
		iir_delayed_y[i] = iir_delayed_y[i-1];
	}

	iir_delayed_x[0] = x_input;
	iir_delayed_y[0] = y_output;

	return y_output;
}
*/


#if defined(HEARTRATE) || defined(PULSE_OX)
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

				if (bio_measurement_type == MEASURE_HR && num_bio_samples <= HR_SAMPLE_NUM)
				{
					#ifdef HEARTRATE
					// Heartrate data collection and filtering
					heartrate_data[HR_SAMPLE_NUM - num_bio_samples] = (q15_t) new_sample[0];
					#endif
				}
				else if (bio_measurement_type == MEASURE_SPO2 && num_bio_samples <= SPO2_SAMPLE_NUM)
				{
					#ifdef PULSE_OX
					// SPO2 data collection and filtering
					spo2_green_data[SPO2_SAMPLE_NUM - num_bio_samples - 1] = (q15_t) new_sample[0];
					spo2_ir_data[SPO2_SAMPLE_NUM - num_bio_samples - 1] = (q15_t) new_sample[1];
					#endif
				}

				//================================displays filtered output===========================
				#ifdef DEBUG_TERMINAL
				char sample_str[15];
				memset(sample_str, 0, 15);
				sprintf(sample_str, "%d\r\n", new_sample[0]);
				//sprintf(sample_str, "%d, %d\r\n", new_sample[0], new_sample[1]);
				HAL_UART_Transmit(&huart1, sample_str, 15, 1000);
				#endif
				//===================================================================================

				num_bio_samples--;
				if (num_bio_samples == 0)
				{
					SPO2_StopMeasurement();

					if (bio_measurement_type == MEASURE_HR)
					{
						#ifdef HEARTRATE
						// Heart rate post processing
						q15_t min = heartrate_data[0];
						for (int i = 1; i < HR_SAMPLE_NUM; i++)
						{
							min = heartrate_data[i] < min ? heartrate_data[i] : min;
						}

						q15_t temp;
						for (int i = 0; i < HR_SAMPLE_NUM; i++)
						{
							//heartrate_data[i] -= min;
							arm_fir_q15(&S, &heartrate_data[i], &temp, 1);

							char str_out[6];
							memset(str_out, 0, 6);
							sprintf(str_out, "%d\r\n", temp);
							HAL_UART_Transmit(&huart1, str_out, 6, 1000);
							//heartrate_data[i] = heartrate_data[i] > 0 ? 0 : heartrate_data[i];
						}

						q15_t rms;
						arm_rms_q15(heartrate_data, HR_SAMPLE_NUM, &rms);
						rms++;
						#endif
					}
					else if (bio_measurement_type == MEASURE_SPO2)
					{
						#ifdef PULSE_OX
						// SPO2 post processing
						q15_t rms_green;
						q15_t rms_ir;
						arm_rms_q15(spo2_green_data, SPO2_SAMPLE_NUM, &rms_green);
						arm_rms_q15(spo2_ir_data, SPO2_SAMPLE_NUM, &rms_ir);
						spo2_percent = (uint8_t) (((float32_t) rms_green) / ((float32_t) rms_ir) * 100);
						#endif
					}

					bio_measurement_flag = FINISHED;
				}
				break;

			case A_FULL_MASK:
				break;

			case DIE_TEMP_RDY_MASK:
				break;

			default:
				break;
		}
	}

	process_bio_data_flag = 0;
}
#endif	/*defined(HEARTRATE) || defined(PULSE_OX)*/


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
#endif	/*DATE_TIME*/


#ifndef USB_DRIVE
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

		case 2:
			HAL_GPIO_WritePin(GAIN_9DB_GPIO_Port, GAIN_9DB_Pin, GPIO_PIN_SET);
			break;

		case 3:
			HAL_GPIO_WritePin(GAIN_12DB_GPIO_Port, GAIN_12DB_Pin, GPIO_PIN_SET);
			break;

		case 4:
			HAL_GPIO_WritePin(GAIN_15DB_GPIO_Port, GAIN_15DB_Pin, GPIO_PIN_SET);
			break;

		default:
			break;
	}
}
#endif	/*USB_DRIVE*/


#if defined(AUDIO) || defined(VIBRATION)
void start_user_feedback()
{
	user_feedback_flag = RUNNING;

	if (watch_settings.feedback_mode == AUDIO_MODE)
	{
		#ifdef AUDIO
		Flash_ReleasePowerDown();	// Wakes flash memory up from power down mode
		HAL_GPIO_WritePin(SD_MODE_GPIO_Port, SD_MODE_Pin, GPIO_PIN_SET);
		audio_buffers = (uint16_t*) calloc(AUDIO_BUFFER_SAMPLES * 2 * NUM_AUDIO_BUFFERS, sizeof(uint16_t));
		PlayerFSM_Prime(audio_list, audio_list_size, audio_buffers);	// Need to add error handling to this
		HAL_I2S_Transmit_DMA(&hi2s2, audio_buffers, AUDIO_BUFFER_SAMPLES * 2 * NUM_AUDIO_BUFFERS);
		#endif
	}
	else if (watch_settings.feedback_mode == VIBRATION_MODE)
	{
		#ifdef VIBRATION
		MorseFSM_Prime(morse_buffer, watch_settings.vibration_length);
		HAL_TIM_Base_Start_IT(&htim21);
		#endif
	}
}


void end_user_feedback()
{
	user_feedback_flag = NOT_STARTED;

	if (watch_settings.feedback_mode == AUDIO_MODE)
	{
		#ifdef AUDIO
		free(audio_buffers);
		#endif
	}
}
#endif	/*defined(AUDIO) || defined(VIBRATION)*/


#ifndef USB_DRIVE
void handle_events()	// Must be called repeatedly in main loop
{
	EWatchEvent event = EventQueue_Peek();

	switch (event)
	{
		#ifdef DATE_TIME
		case GET_TIME_FULL:
		case GET_TIME_MIN:
			switch (user_feedback_flag)
			{
				case NOT_STARTED:
					// Clears buffers
					audio_list_size = 0;
					memset(audio_list, 0, MAX_AUDIO_LIST_SIZE);
					memset(morse_buffer, 0, MAX_MORSE_BUFFER_SIZE);

					// Gets current time from RTC
					RTC_TimeTypeDef current_time;
					RTC_DateTypeDef current_date;
					if (HAL_RTC_GetTime(&hrtc, &current_time, RTC_FORMAT_BIN) != HAL_OK || HAL_RTC_GetDate(&hrtc, &current_date, RTC_FORMAT_BIN) != HAL_OK)
					{
						Error_Handler();
					}

					// Creates feedback for the hour (if desired)
					//if (event == GET_TIME_FULL)
					//{
					uint8_t twelve_hour = convert_24hr_to_12hr(current_time.Hours);

					if (watch_settings.time_format == TIME_TWELVE_HOUR)
					{
						generate_number_audio(audio_list, &audio_list_size, (twelve_hour & 0x0F), 0);
					}
					else
					{
						generate_number_audio(audio_list, &audio_list_size, current_time.Hours, 0);
					}

					generate_number_morse(morse_buffer, current_time.Hours);
					//}

					// Creates feedback for the minutes
					if (current_time.Minutes != 0)
					{
						generate_number_audio(audio_list, &audio_list_size, current_time.Minutes, 1);
					}
					generate_number_morse(morse_buffer, current_time.Minutes);

					// Adds AM/PM if time format is twelve-hour
					if (watch_settings.time_format == TIME_TWELVE_HOUR)
					{
						audio_list[audio_list_size++] = (twelve_hour & 0x80 ? PM_AUDIO : AM_AUDIO);
					}

					start_user_feedback();
					break;

				case FINISHED:
					end_user_feedback();
					HAL_GPIO_WritePin(SD_MODE_GPIO_Port, SD_MODE_Pin, GPIO_PIN_RESET);	// Resets the SM_MODE pin in case the feedback mode was AUDIO
					EventQueue_Dequeue();
					break;

				default:
					break;
			}
			break;

		case GET_DATE_FULL:
		case GET_DATE_DAY:
			switch (user_feedback_flag)
			{
				case NOT_STARTED:
					// Clears buffers
					audio_list_size = 0;
					memset(audio_list, 0, MAX_AUDIO_LIST_SIZE);
					memset(morse_buffer, 0, MAX_MORSE_BUFFER_SIZE);

					// Gets current date from RTC
					RTC_DateTypeDef current_date;
					if (HAL_RTC_GetDate(&hrtc, &current_date, RTC_FORMAT_BIN) != HAL_OK)
					{
						Error_Handler();
					}

					// Creates feedback for the weekday (if desired)
					if (event == GET_DATE_FULL)
					{
						audio_list[audio_list_size++] = SUNDAY_AUDIO + current_date.WeekDay;
						//generate_number_audio(audio_list, &audio_list_size, current_date.WeekDay, 0);	// Change this if we have weekday audio (MONDAY_AUDIO + current_date.WeekDay)
						generate_number_morse(morse_buffer, current_date.WeekDay);
					}

					// Creates feedback for the month and day
					if (watch_settings.date_format == DATE_MDY)
					{
						// Creates feedback for month
						audio_list[audio_list_size++] = JANUARY_AUDIO + current_date.Month - 1;
						generate_number_morse(morse_buffer, current_date.Month);

						// Creates feedback for day
						generate_number_audio(audio_list, &audio_list_size, current_date.Date, 0);
						generate_number_morse(morse_buffer, current_date.Date);
					}
					else
					{
						// Creates feedback for day
						generate_number_audio(audio_list, &audio_list_size, current_date.Date, 0);
						generate_number_morse(morse_buffer, current_date.Date);

						// Creates feedback for month
						audio_list[audio_list_size++] = JANUARY_AUDIO + current_date.Month - 1;
						generate_number_morse(morse_buffer, current_date.Month);
					}

					// Creates feedback for the year (if desired)
					if (event == GET_DATE_FULL)
					{
						audio_list[audio_list_size++] = TWENTY_AUDIO;
						generate_number_audio(audio_list, &audio_list_size, current_date.Year, 0);
						generate_number_morse(morse_buffer, current_date.Year);
					}

					start_user_feedback();
					break;

				case FINISHED:
					end_user_feedback();
					EventQueue_Dequeue();
					break;

				default:
					break;
			}
			break;
		#endif	/*DATE_TIME*/


		case GET_HR:
			#ifdef HEARTRATE
			switch (bio_measurement_flag)
			{
				case NOT_STARTED:
					num_bio_samples = HR_SAMPLE_NUM+1;
					bio_measurement_type = MEASURE_HR;
					heartrate_data = (q15_t*) calloc(HR_SAMPLE_NUM, sizeof(q15_t));
					SPO2_StartMeasurement(bio_measurement_type);
					bio_measurement_flag = RUNNING;
					break;

				case FINISHED:
					// Send results to audio player
					free(heartrate_data);
					bio_measurement_flag = NOT_STARTED;
					EventQueue_Dequeue();
					break;

				default:
					break;
			}
			#else
			EventQueue_Dequeue();
			#endif	/*HEARTRATE*/
			break;

		case GET_PO:
			#ifdef PULSE_OX
			switch (bio_measurement_flag)
			{
				case NOT_STARTED:
					num_bio_samples = SPO2_SAMPLE_NUM+1;
					bio_measurement_type = MEASURE_SPO2;
					spo2_green_data = (q15_t*) calloc(SPO2_SAMPLE_NUM, sizeof(q15_t));
					spo2_ir_data = (q15_t*) calloc(SPO2_SAMPLE_NUM, sizeof(q15_t));
					SPO2_StartMeasurement(bio_measurement_type);
					bio_measurement_flag = RUNNING;
					break;

				case FINISHED:
					switch (user_feedback_flag)
					{
						case NOT_STARTED:
							free(spo2_green_data);
							free(spo2_ir_data);

							// Clears buffers
							audio_list_size = 0;
							memset(audio_list, 0, MAX_AUDIO_LIST_SIZE);
							memset(morse_buffer, 0, MAX_MORSE_BUFFER_SIZE);

							// Generate SPO2 audio
							generate_number_audio(audio_list, &audio_list_size, spo2_percent, 0);
							audio_list[audio_list_size++] = PERCENT_AUDIO;
							generate_number_morse(morse_buffer, spo2_percent);
							start_user_feedback();
							break;

						case FINISHED:
							end_user_feedback();
							bio_measurement_flag = NOT_STARTED;
							EventQueue_Dequeue();
							break;
					}
					break;

				default:
					break;
			}
			#else
			EventQueue_Dequeue();
			#endif	/*PULSE_OX*/
			break;

		case VOL_UP:
			if (watch_settings.feedback_mode == AUDIO_MODE && watch_settings.volume_level < 4)
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
			switch (user_feedback_flag)
			{
				case NOT_STARTED:
					memset(morse_buffer, 0, MAX_MORSE_BUFFER_SIZE);
					morse_buffer[0] = '-';
					start_user_feedback();
					break;

				case FINISHED:
					end_user_feedback();
					EventQueue_Dequeue();
					break;
			}
			break;

		case VOL_UNMUTE:
			watch_settings.feedback_mode = AUDIO_MODE;
			EventQueue_Dequeue();
			break;

		case MENU_UP:
		case MENU_DOWN:
		case MENU_SELECT:
		case MENU_HOME:
			if (watch_settings.feedback_mode == AUDIO_MODE)		// You can only edit settings when not in silent mode
			{
				switch (user_feedback_flag)
				{
					case NOT_STARTED:
						audio_list_size = 0;
						memset(audio_list, 0, MAX_AUDIO_LIST_SIZE);

						run_menu_fsm(event);
						start_user_feedback();
						break;

					case FINISHED:
						end_user_feedback();
						EventQueue_Dequeue();
						break;
				}
				break;
			}
			else
			{
				EventQueue_Dequeue();
			}

		#ifdef CHARGING_MONITOR
		case CHARGE_CYCLE_START:
			HAL_UART_Transmit(&huart1, "Charge Start\r\n", 15, 1000);
			EventQueue_Dequeue();
			break;

		case CHARGE_CYCLE_END:
			HAL_UART_Transmit(&huart1, "Charge End\r\n", 13, 1000);
			EventQueue_Dequeue();
			break;
		#endif	/*CHARGING_MONITOR*/

		#ifdef MEASURE_BATT
		case LOW_BATT_DETECT:
			switch (user_feedback_flag)
			{
				case NOT_STARTED:
					audio_list_size = 0;
					memset(audio_list, 0, MAX_AUDIO_LIST_SIZE);

					audio_list[audio_list_size++] = LOW_AUDIO;
					audio_list[audio_list_size++] = BATTERY_AUDIO;
					start_user_feedback();
					break;

				case FINISHED:
					end_user_feedback();
					EventQueue_Dequeue();
					break;
			}
			break;
		#endif	/*MEASURE_BATT*/

		default:	// Runs if there are no events to handle
			#ifdef LOW_POWER_MODE
			if (!pending_event_flag)	// Prevents us from going back to sleep if we are checking for an event (e.g. taking ADC sample, debouncing, etc.)
			{
				HAL_SuspendTick();
				HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
				SystemClock_Config();
				HAL_ResumeTick();
			}
			#endif	/*LOW_POWER_MODE*/
			break;
	}

}


void run_menu_fsm(EWatchEvent menu_action)
{
	switch (menu_state)
	{
		case MENU_IDLE:
			if (menu_action == MENU_SELECT)
			{
				menu_state = MENU_SET_TIME;
				audio_list[audio_list_size++] = SET_AUDIO;
				audio_list[audio_list_size++] = TIME_AUDIO;
			}
			break;

		case MENU_SET_TIME:
			switch (menu_action) {
				case MENU_SELECT:
					menu_state = MENU_SET_HOUR;
					if (HAL_RTC_GetTime(&hrtc, &set_time, RTC_FORMAT_BIN) != HAL_OK || HAL_RTC_GetDate(&hrtc, &set_date, RTC_FORMAT_BIN) != HAL_OK)
					{
						Error_Handler();
					}
					audio_list[audio_list_size++] = SET_AUDIO;
					audio_list[audio_list_size++] = HOUR_AUDIO;
					break;

				case MENU_UP:
					menu_state = MENU_SET_DATE;
					audio_list[audio_list_size++] = SET_AUDIO;
					audio_list[audio_list_size++] = DATE_AUDIO;
					break;

				case MENU_DOWN:
					menu_state = MENU_FORMAT_TIME;
					audio_list[audio_list_size++] = SET_AUDIO;
					audio_list[audio_list_size++] = TIME_AUDIO;
					audio_list[audio_list_size++] = FORMAT_AUDIO;
					break;
			}
			break;

		case MENU_SET_HOUR:
			switch (menu_action) {
				case MENU_SELECT:
					menu_state = MENU_SET_MINUTE;
					audio_list[audio_list_size++] = SET_AUDIO;
					audio_list[audio_list_size++] = MINUTE_AUDIO;
					break;

				case MENU_UP:
					set_time.Hours = (set_time.Hours >= 23 ? 0 : set_time.Hours + 1);
					generate_number_audio(audio_list, &audio_list_size, set_time.Hours, 0);
					break;

				case MENU_DOWN:
					set_time.Hours = (set_time.Hours == 0 ? 23 : set_time.Hours - 1);
					generate_number_audio(audio_list, &audio_list_size, set_time.Hours, 0);
					break;
			}
			break;

		case MENU_SET_MINUTE:
			switch (menu_action) {
				case MENU_SELECT:
					menu_state = MENU_IDLE;
					set_time.Seconds = 0;
					set_time.SubSeconds = 0;

					if (HAL_RTC_SetTime(&hrtc, &set_time, RTC_FORMAT_BIN) != HAL_OK)
					{
						Error_Handler();
					}

					// Reseting battery check alarm
					#ifdef MEASURE_BATT
					RTC_AlarmTypeDef alarm;
					if (HAL_RTC_GetAlarm(&hrtc, &alarm, RTC_ALARM_A, RTC_FORMAT_BIN) != HAL_OK)
					{
						Error_Handler();
					}

					alarm.AlarmTime.Hours = (set_time.Hours + alarm_hour_inc) % 24;
					alarm.AlarmTime.Minutes = (set_time.Minutes + alarm_min_inc) % 60;
					alarm.AlarmTime.Seconds = (set_time.Seconds + alarm_sec_inc) % 60;

					if (HAL_RTC_SetAlarm_IT(&hrtc, &alarm, RTC_FORMAT_BIN) != HAL_OK)
					{
						Error_Handler();
					}
					#endif

					audio_list[audio_list_size++] = TIME_AUDIO;
					audio_list[audio_list_size++] = SET_AUDIO;
					break;

				case MENU_UP:
					set_time.Minutes = (set_time.Minutes >= 59 ? 0 : set_time.Minutes + 1);
					generate_number_audio(audio_list, &audio_list_size, set_time.Minutes, 0);
					break;

				case MENU_DOWN:
					set_time.Minutes = (set_time.Minutes == 0 ? 59 : set_time.Minutes - 1);
					generate_number_audio(audio_list, &audio_list_size, set_time.Minutes, 0);
					break;
			}
			break;

		case MENU_SET_DATE:
			switch (menu_action) {
				case MENU_SELECT:
					menu_state = MENU_SET_YEAR;
					if (HAL_RTC_GetDate(&hrtc, &set_date, RTC_FORMAT_BIN) != HAL_OK)
					{
						Error_Handler();
					}
					audio_list[audio_list_size++] = SET_AUDIO;
					audio_list[audio_list_size++] = YEAR_AUDIO;
					break;

				case MENU_UP:
					menu_state = MENU_SET_VIB_LEN;
					audio_list[audio_list_size++] = SET_AUDIO;
					audio_list[audio_list_size++] = VIBRATION_AUDIO;
					audio_list[audio_list_size++] = LENGTH_AUDIO;
					break;

				case MENU_DOWN:
					menu_state = MENU_SET_TIME;
					audio_list[audio_list_size++] = SET_AUDIO;
					audio_list[audio_list_size++] = TIME_AUDIO;
					break;
			}
			break;

		case MENU_SET_DAY:
			switch (menu_action) {
				case MENU_SELECT:
					menu_state = MENU_IDLE;

					// Determines day of the week for the month using Zeller's Congruence algorithm
					uint8_t month_code = set_date.Month < 3 ? set_date.Month + 12 : set_date.Month;
					uint8_t weekday_code = (set_date.Date + (13 * (month_code+1) / 5) + set_date.Year) % 7;
					set_date.WeekDay = (weekday_code + 5) % 7;

					if (HAL_RTC_SetDate(&hrtc, &set_date, RTC_FORMAT_BIN) != HAL_OK)
					{
						Error_Handler();
					}

					audio_list[audio_list_size++] = DATE_AUDIO;
					audio_list[audio_list_size++] = SET_AUDIO;
					break;

				case MENU_UP:
				case MENU_DOWN:
					uint8_t upper_lim;
					switch (set_date.Month)
					{
						case 1:
						case 3:
						case 5:
						case 7:
						case 8:
						case 10:
						case 12:
							upper_lim = 31;
							break;

						case 2:
							upper_lim = (set_date.Year % 4 == 0 ? 29 : 28);
							break;

						case 4:
						case 6:
						case 9:
						case 11:
							upper_lim = 30;
							break;

						default:
							upper_lim = 31;
					}

					set_date.Date = (menu_action == MENU_UP ? (set_date.Date >= upper_lim ? 1 : set_date.Date + 1) : (set_date.Date <= 1 ? upper_lim : set_date.Date - 1));
					generate_number_audio(audio_list, &audio_list_size, set_date.Date, 0);
					break;
			}
			break;

		case MENU_SET_MONTH:
			switch (menu_action) {
				case MENU_SELECT:
					menu_state = MENU_SET_DAY;
					audio_list[audio_list_size++] = SET_AUDIO;
					audio_list[audio_list_size++] = DAY_AUDIO;
					break;

				case MENU_UP:
					set_date.Month = (set_date.Month >= 12 ? 1 : set_date.Month + 1);
					audio_list[audio_list_size++] = JANUARY_AUDIO + set_date.Month - 1;
					break;

				case MENU_DOWN:
					set_date.Month = (set_date.Month <= 1 ? 12 : set_date.Month - 1);
					audio_list[audio_list_size++] = JANUARY_AUDIO + set_date.Month - 1;
					break;
			}
			break;

		case MENU_SET_YEAR:
			switch (menu_action) {
				case MENU_SELECT:
					menu_state = MENU_SET_MONTH;
					audio_list[audio_list_size++] = SET_AUDIO;
					audio_list[audio_list_size++] = MONTH_AUDIO;
					break;

				case MENU_UP:
					set_date.Year = (set_date.Year >= 99 ? 0 : set_date.Year + 1);
					generate_number_audio(audio_list, &audio_list_size, set_date.Year, 0);
					break;

				case MENU_DOWN:
					set_date.Year = (set_date.Year == 0 ? 99 : set_date.Year - 1);
					generate_number_audio(audio_list, &audio_list_size, set_date.Year, 0);
					break;
			}
			break;

		case MENU_FORMAT_TIME:
			switch (menu_action) {
				case MENU_SELECT:
					menu_state = MENU_SET_FORMAT_TIME;
					generate_number_audio(audio_list, &audio_list_size, (watch_settings.time_format == TIME_TWELVE_HOUR ? 12 : 24), 0);
					break;

				case MENU_UP:
					menu_state = MENU_SET_TIME;
					audio_list[audio_list_size++] = SET_AUDIO;
					audio_list[audio_list_size++] = TIME_AUDIO;
					break;

				case MENU_DOWN:
					menu_state = MENU_FORMAT_DATE;
					audio_list[audio_list_size++] = SET_AUDIO;
					audio_list[audio_list_size++] = DATE_AUDIO;
					audio_list[audio_list_size++] = FORMAT_AUDIO;
					break;
			}
			break;

		case MENU_FORMAT_DATE:
			switch (menu_action) {
				case MENU_SELECT:
					menu_state = MENU_SET_FORMAT_DATE;
					if (watch_settings.date_format == DATE_MDY)
					{
						audio_list[audio_list_size++] = MONTH_AUDIO;
						audio_list[audio_list_size++] = DAY_AUDIO;
					}
					else
					{
						audio_list[audio_list_size++] = DAY_AUDIO;
						audio_list[audio_list_size++] = MONTH_AUDIO;
					}

					break;

				case MENU_UP:
					menu_state = MENU_FORMAT_TIME;
					audio_list[audio_list_size++] = SET_AUDIO;
					audio_list[audio_list_size++] = TIME_AUDIO;
					audio_list[audio_list_size++] = FORMAT_AUDIO;
					break;

				case MENU_DOWN:
					menu_state = MENU_SET_VIB_LEN;
					audio_list[audio_list_size++] = SET_AUDIO;
					audio_list[audio_list_size++] = VIBRATION_AUDIO;
					audio_list[audio_list_size++] = LENGTH_AUDIO;
					break;
			}
			break;

		case MENU_SET_VIB_LEN:
			switch (menu_action) {
				case MENU_SELECT:
					menu_state = MENU_CHANGE_VIB_LENGTH;
					audio_list[audio_list_size++] = SHORT_AUDIO + watch_settings.vibration_length - 1;
					break;

				case MENU_UP:
					menu_state = MENU_FORMAT_DATE;
					audio_list[audio_list_size++] = SET_AUDIO;
					audio_list[audio_list_size++] = DATE_AUDIO;
					audio_list[audio_list_size++] = FORMAT_AUDIO;
					break;

				case MENU_DOWN:
					menu_state = MENU_SET_DATE;
					audio_list[audio_list_size++] = SET_AUDIO;
					audio_list[audio_list_size++] = DATE_AUDIO;
					break;
			}
			break;

		case MENU_SET_FORMAT_TIME:
			switch (menu_action) {
				case MENU_SELECT:
					menu_state = MENU_IDLE;
					audio_list[audio_list_size++] = TIME_AUDIO;
					audio_list[audio_list_size++] = FORMAT_AUDIO;
					audio_list[audio_list_size++] = SET_AUDIO;
					break;

				case MENU_UP:
				case MENU_DOWN:
					watch_settings.time_format = (watch_settings.time_format == TIME_TWELVE_HOUR ? TIME_TWENTYFOUR_HOUR : TIME_TWELVE_HOUR);
					generate_number_audio(audio_list, &audio_list_size, (watch_settings.time_format == TIME_TWELVE_HOUR ? 12 : 24), 0);
					break;
			}
			break;

		case MENU_SET_FORMAT_DATE:
			switch (menu_action) {
				case MENU_SELECT:
					menu_state = MENU_IDLE;
					audio_list[audio_list_size++] = DATE_AUDIO;
					audio_list[audio_list_size++] = FORMAT_AUDIO;
					audio_list[audio_list_size++] = SET_AUDIO;
					break;

				case MENU_UP:
				case MENU_DOWN:
					watch_settings.date_format = (watch_settings.date_format == DATE_MDY ? DATE_DMY : DATE_MDY);

					if (watch_settings.date_format == DATE_MDY)
					{
						audio_list[audio_list_size++] = MONTH_AUDIO;
						audio_list[audio_list_size++] = DAY_AUDIO;
					}
					else
					{
						audio_list[audio_list_size++] = DAY_AUDIO;
						audio_list[audio_list_size++] = MONTH_AUDIO;
					}
					break;
			}
			break;

		case MENU_CHANGE_VIB_LENGTH:
			switch (menu_action) {
				case MENU_SELECT:
					menu_state = MENU_IDLE;
					audio_list[audio_list_size++] = VIBRATION_AUDIO;
					audio_list[audio_list_size++] = LENGTH_AUDIO;
					audio_list[audio_list_size++] = SET_AUDIO;
					break;

				case MENU_UP:
					if (watch_settings.vibration_length < 3)
					{
						watch_settings.vibration_length++;
						audio_list[audio_list_size++] = SHORT_AUDIO + watch_settings.vibration_length - 1;
					}
					break;

				case MENU_DOWN:
					if (watch_settings.vibration_length > 1)
					{
						watch_settings.vibration_length--;
						audio_list[audio_list_size++] = SHORT_AUDIO + watch_settings.vibration_length - 1;
					}
					break;
			}
			break;
	}

	if (menu_action == MENU_HOME)
	{
		menu_state = MENU_IDLE;
	}
}
#endif	/*USB_DRIVE*/


#ifndef USB_DRIVE
void MX_USB_PCD_Init() {}	// Dummy instance of this function to keep the compiler happy
#endif	/*USB_DRIVE*/


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

  #ifdef SPI_FLASH
  if (Flash_Init(&hspi1, FLASH_CS_GPIO_Port, FLASH_CS_Pin) != FLASH_OK)
  {
	  Error_Handler();
  }

  #ifndef USB_DRIVE
  if (Flash_PowerDown() != FLASH_OK)
  {
	  Error_Handler();
  }
  #endif	/*USB_DRIVE*/
  #endif	/*SPI_FLASH*/

  #if defined(HEARTRATE) || defined(PULSE_OX)
  if (SPO2_Init(&hi2c1) != SPO2_OK)
  {
	  Error_Handler();
  }

  arm_fir_init_q15(&S, FIR_FILTER_TAPS, FIR_COEFFICIENTS, fir_state, 1);
  #endif	/*defined(HEARTRATE) || defined(PULSE_OX)*/

  #ifdef MEASURE_BATT
  HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);

  RTC_AlarmTypeDef batt_meas_alarm;

  if (HAL_RTC_GetAlarm(&hrtc, &batt_meas_alarm, RTC_ALARM_A, RTC_FORMAT_BIN) != HAL_OK)
  {
	  Error_Handler();
  }

  alarm_hour_inc = batt_meas_alarm.AlarmTime.Hours;
  alarm_min_inc = batt_meas_alarm.AlarmTime.Minutes;
  alarm_sec_inc = batt_meas_alarm.AlarmTime.Seconds;
  #endif	/*MEASURE_BATT*/

  #ifndef USB_DRIVE
  set_gain_pins();
  #endif	/*USB_DRIVE*/

  #ifdef EXTI_SIGNALS
  Button_Release();
  #endif	/*EXTI_SIGNALS*/

  #ifdef USB_DRIVE
  tusb_init();
  #endif	/*USB_DRIVE*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  #ifdef USB_DRIVE
	  tud_task();
	  #endif /*USB_DRIVE*/

	  #ifdef DEBUG_TERMINAL
	  #ifdef VIBRATION
	  if (!morse_rx_flag)
	  {
	  #endif	/*VIBRATION*/
	  	  HAL_UART_Receive_IT(&huart1, rx_buffer, 2);
	  #ifdef VIBRATION
	  }
	  #endif	/*VIBRATION*/
	  #endif	/*DEBUG_TERMINAL*/


	  #ifdef AUDIO
	  if (fill_audio_buffer_flag)
	  {
		  fill_audio_buffer();
	  }
 	  #endif	/*AUDIO*/


	  #if defined(HEARTRATE) || defined(PULSE_OX)
	  if (process_bio_data_flag)
	  {
		  process_spo2_data();
	  }
	  #endif	/*defined(HEARTRATE) || defined(PULSE_OX)*/


	  #ifndef USB_DRIVE
	  handle_events();
	  #endif	/*USB_DRIVE*/

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

  #ifdef DEBUG_TERMINAL
  while (1);
  #endif

  NVIC_SystemReset();
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
