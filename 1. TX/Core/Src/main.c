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
#include "BME280/bme280_defs.h"
#include "BME280/bme280.h"

#include "nrf24L01/nrf24L01.h"

#include "DS3231/ds3231.h"

#include "stm32f4xx_hal.h"

#include "stm32f4xx_hal_gpio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ON 1
#define OFF 0

#define SLEEP_MODE ON

#define UART_LOG ON
#define LED ON


#define RED_LED_ON HAL_GPIO_WritePin(GPIOB, LED_Red_Pin, GPIO_PIN_SET)
#define RED_LED_OFF HAL_GPIO_WritePin(GPIOB, LED_Red_Pin, GPIO_PIN_RESET)
#define RED_LED_TOGLE HAL_GPIO_TogglePin(GPIOB, LED_Red_Pin)

#define GREEN_LED_ON HAL_GPIO_WritePin(GPIOB, LED_Green_Pin, GPIO_PIN_SET)
#define GREEN_LED_OFF HAL_GPIO_WritePin(GPIOB, LED_Green_Pin, GPIO_PIN_RESET)
#define GREEN_LED_TOGLE HAL_GPIO_TogglePin(GPIOB, LED_Green_Pin)


// Set wakeUp time
const float  Wake_up_Time_Base = 16.0f/32000.0f;     //0.0005 s
const int WakeUpCounter_1_sec = 1/Wake_up_Time_Base;
const int WakeUpCounter_5_sec = 5/Wake_up_Time_Base;
const int WakeUpCounter_10_sec = 10/Wake_up_Time_Base;



////////////////////////////////////////////////////////////////////////////




// Define wake up period


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */




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




// BME280 part /////////////////////////////////////////////////////////////////////////////////////
struct bme280_dev dev;
struct bme280_data comp_data;
//	int8_t rslt;

int8_t init_bme280(void);
void bme280_measure(void);

//----------------------------------------------------------------------------------------
int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	if(HAL_I2C_Master_Transmit(&hi2c1, (id << 1), &reg_addr, 1, 10) != HAL_OK) return -1;
	if(HAL_I2C_Master_Receive(&hi2c1, (id << 1) | 0x01, data, len, 10) != HAL_OK) return -1;

	return 0;
}
//----------------------------------------------------------------------------------------
void user_delay_ms(uint32_t period)
{
	HAL_Delay(period);
}
//----------------------------------------------------------------------------------------
int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	int8_t *buf;
	buf = malloc(len +1);
	buf[0] = reg_addr;
	memcpy(buf +1, data, len);

	if(HAL_I2C_Master_Transmit(&hi2c1, (id << 1), (uint8_t*)buf, len + 1, HAL_MAX_DELAY) != HAL_OK) return -1;

	free(buf);
	return 0;
}
//----------------------------------------------------------------------------------------
void init_bme280_(void)
{
	uint16_t STATUS=0;
	uint16_t addres_device = 0x76;  		 		// BME280
	uint16_t id_addr = 0xD0;
	uint8_t id = 96;								// in hex form
	uint8_t buff=0;        						// Return 0x96 -> Dec 60
	int8_t rslt;

	// For debug
	STATUS = HAL_I2C_Mem_Read(&hi2c1, addres_device<<1, id_addr, 1, &buff, 1, 1000);
	if(!((buff == id) && (STATUS == 0)))
	{
		error_blink_red_led();
	}

	// Init BME280
	dev.dev_id = BME280_I2C_ADDR_PRIM;
	dev.intf = BME280_I2C_INTF;
	dev.read = user_i2c_read;
	dev.write = user_i2c_write;
	dev.delay_ms = user_delay_ms;

	rslt = bme280_init(&dev);

	dev.settings.osr_h = BME280_OVERSAMPLING_1X;
	dev.settings.osr_p = BME280_OVERSAMPLING_16X;
	dev.settings.osr_t = BME280_OVERSAMPLING_2X;
	dev.settings.filter = BME280_FILTER_COEFF_16;
	rslt = bme280_set_sensor_settings(BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL, &dev);
	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev);

	dev.delay_ms(40);
}
//----------------------------------------------------------------------------------------
void get_THP_bme280(float *T, float *H, float *P)
{
	if(bme280_get_sensor_data(BME280_ALL, &comp_data, &dev) == BME280_OK)
	{
		// Save data variables
		*T = (float)comp_data.temperature;
		*H = (float)comp_data.humidity;
		*P = (float)comp_data.pressure;
	}
	else
	{
		error_blink_red_led();
	}

	if(put_device_to_sleep(&dev) != 0)
	{
		error_blink_red_led();
	}
}
// End BME280 part/////////////////////////////////////////////////////////////////////////////////////
// --------------------------------------------------------------------------------
void meassure_battery_voltage(float *voltage)
{
	uint16_t i = 0;
	char str[35]={0};

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
//		i = (uint32_t)HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	float V_bat = (float)HAL_ADC_GetValue(&hadc1)*3.3/4096;

	*voltage = V_bat;

//		sprintf(str, "ADC: %04d V_bat: %.2f V \n\r", i, V_bat);
//		HAL_UART_Transmit(&huart1, str, sizeof(str), 1000);
}
// --------------------------------------------------------------------------------
void led_test_blink(void)
{
	for(int i = 0; i <= 5; i++)
	{
		RED_LED_ON;
		GREEN_LED_ON;
		HAL_Delay(100);
		RED_LED_OFF;
		GREEN_LED_OFF;
		HAL_Delay(100);
	}
}
// --------------------------------------------------------------------------------------
void error_blink_red_led(void)
{
	while(1)
	{
		RED_LED_TOGLE;
		HAL_Delay(100);
	}
}
// --------------------------------------------------------------------------------------
#define COUNTER_PACKET 1
#define COUNTER_RETRANSMITED_PACKET 2
#define COUNTER_LOST_PACKET 3

void WriteBackupRegister(uint32_t data, uint8_t type)
{
	if(type == COUNTER_PACKET)
	{
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, data);
	}
}
// --------------------------------------------------------------------------------
uint32_t ReadBackupRegister(uint8_t type)
{
	if(type == COUNTER_PACKET)
	{
		return HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);
	}
}
// --------------------------------------------------------------------------------

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

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////

  if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET)
  {
	  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);  // clear the flag

#if LED == ON
	  GREEN_LED_ON;
#endif

	  float BME280_temperature = 0;
	  float BME280_humidity = 0;
	  float BME280_preasure = 0;

	  float battery_voltage = 0;

	  init_bme280_();
	  get_THP_bme280(&BME280_temperature, &BME280_humidity, &BME280_preasure);

 	  //  Prepare string of data
 	  uint8_t num_of_dev = 1;
 	  char tramsmeet_data_buffer[50] = {0,};

 	  char buf[10] = {0,};

 	  memset(buf, 0, sizeof(buf));
 	  sprintf(buf, "T%.1f", BME280_temperature);
 	  strcat(tramsmeet_data_buffer, buf);

 	  memset(buf, 0, sizeof(buf));
 	  sprintf(buf, "H%.1f", BME280_humidity);
 	  strcat(tramsmeet_data_buffer, buf);

 	  // Meassure voltage on battery
 	  meassure_battery_voltage(&battery_voltage);
 	  memset(buf, 0, sizeof(buf));
 	  sprintf(buf, "V%.1f ", battery_voltage);
 	  strcat(tramsmeet_data_buffer, buf);

 	  // DS3231 clock
 	  uint8_t buffer = 0;
 	  uint8_t time_ds3231[10] = {0,};

 	  for(int i = 0; i <=8; i++)
 	  {
 		  ds3231_read(i, &buffer);
 		  time_ds3231[i] = buffer;
 	  }

 	  // convert time into string
 	  char buf_1[5] = {0,};

 	  for(int i = 2; i >= 0 ; i--)
 	  {
 		  sprintf(buf_1, "%d", time_ds3231[i]);
 		  strcat(tramsmeet_data_buffer, buf_1);
 		  memset(buf_1, 0, sizeof(buf_1));
 		  if(i > 0)
 		  {
 			  strcat(tramsmeet_data_buffer, ":");
 		  }
 	  }
 	  strcat(tramsmeet_data_buffer, " ");

#if UART_LOG == ON
 	  char buf_uart_tx[70] = {0,};
 	  sprintf(buf_uart_tx, "TX data: %s \n\r", tramsmeet_data_buffer);

 	  HAL_UART_Transmit(&huart1, buf_uart_tx, sizeof(buf_uart_tx), 1000);
#endif

 	  NRF24_init_TX(0, 10, 0, 15, 0, 0);

 	  // Detect loat packages
	  char buf1[10] = {0,};
	  char buf2[54] = {0,};

	  uint8_t retr_packages = 0;
	  uint8_t dt = 0 ;
 	  uint16_t lost_packages = 0;


 	  uint32_t RTC_DATA = ReadBackupRegister(COUNTER_PACKET);
 	  memset(buf_1, 0, sizeof(buf_1));
 	  sprintf(buf_1, "C%d", RTC_DATA);
 	  strcat(tramsmeet_data_buffer, buf_1);


 	  dt = NRF24L01_Transmit(1, tramsmeet_data_buffer);

 	  retr_packages  = dt & 0xF;			// Select retransmit packets
 	  lost_packages = dt & 0xF0;			// Select lost packets



#if UART_LOG == ON
 	  sprintf(buf2, "TX retr: %d, TX lost: %d, COUNTER_PACKET: %d\n\r", retr_packages, lost_packages, RTC_DATA);
 	  HAL_UART_Transmit(&huart1, buf2, sizeof(buf2), 1000);
#endif

 	  WriteBackupRegister(RTC_DATA+1, COUNTER_PACKET);

 	  if(lost_packages > 0)		// If lost packages was detected
 	  {
 		 RED_LED_ON;
 		 HAL_Delay(10);
 		 RED_LED_OFF;
// 		 HAL_UART_Transmit(&huart1, str, sizeof(str), 1000);
 	  }
 	  else
 	  {
 		 RED_LED_OFF;
 	  }

#if LED == ON
 	  GREEN_LED_OFF;
#endif

 	  NRF24_Sleep_mode();


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

   if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, WakeUpCounter_1_sec, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
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
	  float BME280_temperature = 0;
	  float BME280_humidity = 0;
	  float BME280_preasure = 0;

	  float battery_voltage = 0;



	   	 init_bme280_();

	   	 get_THP_bme280(&BME280_temperature, &BME280_humidity, &BME280_preasure);

	   	  //  Prepare string of data
	   	  uint8_t num_of_dev = 1;
	   	  char tramsmeet_data_buffer[40] = {0,};

	   	  char buf[10] = {0,};

	   	  memset(buf, 0, sizeof(buf));
	   	  sprintf(buf, "T%.1f", BME280_temperature);
	   	  strcat(tramsmeet_data_buffer, buf);

	   	  memset(buf, 0, sizeof(buf));
	   	  sprintf(buf, "H%.1f", BME280_humidity);
	   	  strcat(tramsmeet_data_buffer, buf);


	   	  // Meassure voltage on battery
	   	  meassure_battery_voltage(&battery_voltage);
	   	  memset(buf, 0, sizeof(buf));
	   	  sprintf(buf, "V%.1f ", battery_voltage);
	   	  strcat(tramsmeet_data_buffer, buf);

	   	  // DS3231 clock
	   	  uint8_t buffer = 0;
	   	  uint8_t time_ds3231[10] = {0,};

	   	  for(int i = 0; i <=8; i++)
	   	  {
	   		  ds3231_read(i, &buffer);
	   		  time_ds3231[i] = buffer;
	   	  }


	   	  // convert into string
	   	  char buf_1[5] = {0,};

	   	  for(int i = 2; i >= 0 ; i--)
	   	  {
	   		  sprintf(buf_1, "%d", time_ds3231[i]);
	   		  strcat(tramsmeet_data_buffer, buf_1);
	   		  memset(buf_1, 0, sizeof(buf_1));
	   		  if(i > 0)
	   		  {
	   			  strcat(tramsmeet_data_buffer, ":");
	   		  }
	   	  }
	   	  strcat(tramsmeet_data_buffer, " ");


	   	  // HAL_UART_Transmit(&huart1, tramsmeet_data_buffer, sizeof(tramsmeet_data_buffer), 1000);

	   	  NRF24_init_TX(0, 10, 0, 15, 0, 0);

	   	  // Detect loat packages
	  	  char buf1[10] = {0,};
	  	  char buf2[54] = {0,};

	  	  uint8_t retr_packages = 0;;
	  	  uint8_t dt = 0 ;
	   	  uint16_t lost_packages = 0;


	   	  dt = NRF24L01_Transmit(1, tramsmeet_data_buffer);

	   	  retr_packages  = dt & 0xF;			// Select retransmit packets
	   	  lost_packages = dt & 0xF0;			// Select lost packets

	   	  sprintf(buf2, "TX retr: %d, TX lost: %d \n\r", retr_packages, lost_packages);
	   	  HAL_UART_Transmit(&huart1, buf2, sizeof(buf2), 1000);

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


	   	  NRF24_Sleep_mode();

	   	HAL_Delay(1000);
#endif
	  //////////////////// TX ///////////////////////
//	  NRF24L01_Transmit(1, "1234567890");
//	  HAL_Delay(1000);
	  ///////////////////////////////////////////////

	  //////////////////// RX ///////////////////////
//	   NRF24L01_Receive();
	  ///////////////////////////////////////////////

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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
