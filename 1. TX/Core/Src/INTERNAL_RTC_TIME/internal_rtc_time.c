/*
 * internal_rtc_time.c
 *
 *  Created on: Jun 26, 2024
 *      Author: odemki
 */


#include "main.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

#include "INTERNAL_RTC_TIME/internal_rtc_time.h"

extern RTC_HandleTypeDef hrtc;
extern UART_HandleTypeDef huart1;

// --------------------------------------------------------------------------------------
void read_time_internal_RTC(char *buff)
{
	 RTC_TimeTypeDef sTime;
	 RTC_DateTypeDef sDate;

	 HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	 HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);


	 char buf_uart_tx[70] = {0,};
	 sprintf(buf_uart_tx, "%d:%d:%d %d.%d.%d ", sTime.Hours, sTime.Minutes, sTime.Seconds, sDate.Date, sDate.Month, sDate.Year);
	 strcat(buff, buf_uart_tx);

#if UART_LOG == ON
	 HAL_UART_Transmit(&huart1, buf_uart_tx, sizeof(buf_uart_tx), 1000);
#endif
}
// -----------------------------------------------------------------------------------------------
void set_time_internal_RTC(void)
{
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;

	sTime.Hours = 0x0B;				// 11
	sTime.Minutes = 0x0C;			// 12
	sTime.Seconds = 0x0D;			// 13
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
	{
		Error_Handler();
	}

	sDate.WeekDay = RTC_WEEKDAY_MONDAY;
	sDate.Month = RTC_MONTH_JANUARY;
	sDate.Date = 0x1;
	sDate.Year = 0x18;

	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
	{
		Error_Handler();
	}
}
// --------------------------------------------------------------------------------------
