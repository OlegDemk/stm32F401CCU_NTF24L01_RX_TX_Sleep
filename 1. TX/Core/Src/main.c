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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"

#include "BME280/bme280_defs.h"
#include "BME280/bme280.h"
#include "nrf24L01/nrf24L01.h"
#include "DS3231/ds3231.h"
#include "BME280/main_bme280.h"
#include "BATTERY_VOLTAGE/adc_voltage.h"
#include "RTC_DATA/rtc_data_write_read.h"
#include "LEDs/leds.h"
#include "DS3231/main_ds3231.h"
#include "INTERNAL_RTC_TIME/internal_rtc_time.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Set wakeUp time
const float  Wake_up_Time_Base = 16.0f/32000.0f;     //0.0005 s
const int WakeUpCounter_1_sec = 1/Wake_up_Time_Base;
const int WakeUpCounter_5_sec = 5/Wake_up_Time_Base;
const int WakeUpCounter_10_sec = 10/Wake_up_Time_Base;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

void send_data(int delay);

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// --------------------------------------------------------------------------------------
void settings_mode(void)
{


}
// --------------------------------------------------------------------------------------
void counter(char* buff)
{



}
// --------------------------------------------------------------------------------------
void send_data(int delay)
{
	  // If button was presset more than 5 sec, go to settings mode
	if(HAL_GPIO_ReadPin(GPIOA, ACTION_Pin) == 1)
	{
#if UART_LOG == ON
		char buf_uart_tx[50] = {0,};
		sprintf(buf_uart_tx, "Settings mode\n\r");
		HAL_UART_Transmit(&huart1, buf_uart_tx, sizeof(buf_uart_tx), 1000);
#endif
		led_test_blink(10, 800);
		settings_mode();
	}

	char tramsmeet_data_buffer[70] = {0,};
	char buf_uart_tx[50] = {0,};

#if LED == ON
	GREEN_LED_ON;
#endif
	get_THP_bme280(tramsmeet_data_buffer); 				// Meassure T, H and P
	meassure_battery_voltage(tramsmeet_data_buffer);  	// Meassure voltage on battery
	//read_time_internal_RTC(tramsmeet_data_buffer);

#if UART_LOG == ON
	memset(buf_uart_tx, 0, sizeof(buf_uart_tx));
	sprintf(buf_uart_tx, "TX data: %s \n\r", tramsmeet_data_buffer);
	HAL_UART_Transmit(&huart1, buf_uart_tx, sizeof(buf_uart_tx), 1000);
#endif
	uint8_t dt = 0 ;
	char buf_1[25] = {0,};

	uint32_t RTC_retransmit_packets_cpunter = ReadBackupRegister(RTC_COUNTER_RETRANSMITED_PACKET);
	uint32_t RTC_lost_packets_cpunter = ReadBackupRegister(RTC_COUNTER_LOST_PACKET);
	uint32_t RTC_counter = ReadBackupRegister(RTC_COUNTER_PACKET);

	memset(buf_1, 0, sizeof(buf_1));
	sprintf(buf_1, "C%dR%dL%d", RTC_counter, RTC_retransmit_packets_cpunter, RTC_lost_packets_cpunter);
	strcat(tramsmeet_data_buffer, buf_1);

	NRF24_init_TX(0, 10, 0, 15, 0, 0);
	dt = NRF24L01_Transmit(1, tramsmeet_data_buffer);

	uint8_t retr_packages  = dt & 0xF;			// Select retransmit packets
	uint8_t lost_packages = dt & 0xF0;			// Select lost packets

	// Save data into RTC memory
	WriteBackupRegister(retr_packages, RTC_COUNTER_RETRANSMITED_PACKET);
	WriteBackupRegister(lost_packages, RTC_COUNTER_LOST_PACKET);
	WriteBackupRegister(RTC_counter+1, RTC_COUNTER_PACKET);

#if UART_LOG == ON
	memset(buf_uart_tx, 0, sizeof(buf_uart_tx));
	sprintf(buf_uart_tx, "COUNTER_PACKET: %d, TX retr: %d, TX lost: %d, \n\r", RTC_counter, retr_packages, lost_packages);
	HAL_UART_Transmit(&huart1, buf_uart_tx, sizeof(buf_uart_tx), 1000);
#endif
	if(lost_packages > 0)		// If lost packages was detected
	{
		RED_LED_ON;
		HAL_Delay(10);
		RED_LED_OFF;
	}
	else
	{
		RED_LED_OFF;
	}

#if LED == ON
	GREEN_LED_OFF;
#endif
	NRF24_Sleep_mode();

	HAL_Delay(delay);
}
// --------------------------------------------------------------------------------------
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
  MX_RTC_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */


#if SLEEP_MODE == ON
	if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET)			// Is device woke up from standby mode ?
	{
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);  				// clear the flag

		HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);  		// disable PA0
		HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);				// Deactivate the RTC wakeup

		// If button was presset more than 5 sec, go to settings mode
		if(HAL_GPIO_ReadPin(GPIOA, ACTION_Pin) == 1)
		{
	#if UART_LOG == ON
			char buf_uart_tx[50] = {0,};
			sprintf(buf_uart_tx, "Settings mode\n\r");
			HAL_UART_Transmit(&huart1, buf_uart_tx, sizeof(buf_uart_tx), 1000);
	#endif

			led_test_blink(10, 800);
			settings_mode();
		}

		  uint32_t period = ReadBackupRegister(RTC_COUNTER_SECOND);

		  if(period >= SLEEP_TIME)		// Wake up time condition
		  {
			  char tramsmeet_data_buffer[70] = {0,};
			  char buf_uart_tx[50] = {0,};

	#if LED == ON
			  GREEN_LED_ON;
	#endif
			  get_THP_bme280(tramsmeet_data_buffer); 				// Meassure T, H and P
			  meassure_battery_voltage(tramsmeet_data_buffer);  	// Meassure voltage on battery
			  //read_time_internal_RTC(tramsmeet_data_buffer);

	#if UART_LOG == ON
			  memset(buf_uart_tx, 0, sizeof(buf_uart_tx));
			  sprintf(buf_uart_tx, "TX data: %s \n\r", tramsmeet_data_buffer);
			  HAL_UART_Transmit(&huart1, buf_uart_tx, sizeof(buf_uart_tx), 1000);
	#endif
			  uint8_t dt = 0 ;
			  char buf_1[25] = {0,};

			  uint32_t RTC_retransmit_packets_cpunter = ReadBackupRegister(RTC_COUNTER_RETRANSMITED_PACKET);
			  uint32_t RTC_lost_packets_cpunter = ReadBackupRegister(RTC_COUNTER_LOST_PACKET);
			  uint32_t RTC_counter = ReadBackupRegister(RTC_COUNTER_PACKET);

			  memset(buf_1, 0, sizeof(buf_1));
			  sprintf(buf_1, "C%dR%dL%d", RTC_counter, RTC_retransmit_packets_cpunter, RTC_lost_packets_cpunter);
			  strcat(tramsmeet_data_buffer, buf_1);

			  NRF24_init_TX(0, 10, 0, 15, 0, 0);
			  dt = NRF24L01_Transmit(1, tramsmeet_data_buffer);

			  uint8_t retr_packages  = dt & 0xF;			// Select retransmit packets
			  uint8_t lost_packages = dt & 0xF0;			// Select lost packets

			  // Save data into RTC memory
			  WriteBackupRegister(retr_packages, RTC_COUNTER_RETRANSMITED_PACKET);
			  WriteBackupRegister(lost_packages, RTC_COUNTER_LOST_PACKET);
			  WriteBackupRegister(RTC_counter+1, RTC_COUNTER_PACKET);

	#if UART_LOG == ON
			  memset(buf_uart_tx, 0, sizeof(buf_uart_tx));
			  sprintf(buf_uart_tx, "COUNTER_PACKET: %d, TX retr: %d, TX lost: %d, \n\r", RTC_counter, retr_packages, lost_packages);
			  HAL_UART_Transmit(&huart1, buf_uart_tx, sizeof(buf_uart_tx), 1000);
	#endif
			  if(lost_packages > 0)		// If lost packages was detected
			  {
				  RED_LED_ON;
				  HAL_Delay(10);
				  RED_LED_OFF;
			  }
			  else
			  {
				  RED_LED_OFF;
			  }

	#if LED == ON
			  GREEN_LED_OFF;
	#endif
			  NRF24_Sleep_mode();

			  WriteBackupRegister(0, RTC_COUNTER_SECOND);
		  }
		  else
		  {
			  WriteBackupRegister(period+1, RTC_COUNTER_SECOND);		// Update counter of seconds
		  }
		  /** Disable the WWAKEUP PIN **/
		  HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);  // disable PA0
		  /** Deactivate the RTC wakeup  **/
		  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
	   }


	  /** Now enter the standby mode **/
		/* Clear the WU FLAG */
	   __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

		/* clear the RTC Wake UP (WU) flag */
	   __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);


		/* Enable the WAKEUP PIN */
	   HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);

	   if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, WakeUpCounter_5_sec, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
	   {
		 Error_Handler();
	   }

	   /* Finally enter the standby mode */
	   HAL_PWR_EnterSTANDBYMode();

#else





#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#if SLEEP_MODE == OFF
	  send_data(100);
#endif

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
  if(__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) == RESET)		// If RTC doesn't set
  {
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x01;
  sTime.Minutes = 0x02;
  sTime.Seconds = 0x03;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x17;
  sDate.Year = 0x24;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  }
  else
  {

  }
  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, nrf_CE_Pin|nrf_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Red_Pin|LED_Green_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : nrf_IRQ_Pin */
  GPIO_InitStruct.Pin = nrf_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(nrf_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : nrf_CE_Pin nrf_CS_Pin */
  GPIO_InitStruct.Pin = nrf_CE_Pin|nrf_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ACTION_Pin */
  GPIO_InitStruct.Pin = ACTION_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ACTION_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Red_Pin LED_Green_Pin */
  GPIO_InitStruct.Pin = LED_Red_Pin|LED_Green_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
