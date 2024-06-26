/*
 * adc_voltage.c
 *
 *  Created on: Jun 24, 2024
 *      Author: odemki
 */

#include "main.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;
extern ADC_HandleTypeDef hadc1;

// --------------------------------------------------------------------------------
void meassure_battery_voltage(char * buff)
{
	uint16_t i = 0;
	char buffer[10] = {0,};

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
//		i = (uint32_t)HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	float V_bat = (float)HAL_ADC_GetValue(&hadc1)*3.3/4096;

	memset(buffer, 0, sizeof(buffer));
	sprintf(buffer, "V%.1f ", V_bat);
	strcat(buff, buffer);
}
// --------------------------------------------------------------------------------
