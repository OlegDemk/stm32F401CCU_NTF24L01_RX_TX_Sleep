/*
 * rtc_data_write_read.c
 *
 *  Created on: Jun 24, 2024
 *      Author: odemki
 */

#include "main.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

#include "RTC_DATA/rtc_data_write_read.h"

extern RTC_HandleTypeDef hrtc;

static const uint32_t RTC_BKP_DR[] = {RTC_BKP_DR1, RTC_BKP_DR2, RTC_BKP_DR3, RTC_BKP_DR4};

void WriteBackupRegister(uint32_t data, uint8_t type)
{
	if(type >= RTC_COUNTER_PACKET && type <= RTC_COUNTER_LOST_PACKET)
	{
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR[type - 1], data);
	}
}
// --------------------------------------------------------------------------------
uint32_t ReadBackupRegister(uint8_t type)
{
	if(type >= RTC_COUNTER_PACKET && type <= RTC_COUNTER_LOST_PACKET)
	{
		return HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR[type - 1]);
	}
}
// --------------------------------------------------------------------------------
