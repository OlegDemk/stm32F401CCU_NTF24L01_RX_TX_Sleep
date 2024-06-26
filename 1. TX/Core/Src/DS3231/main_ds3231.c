/*
 * main_ds3231.c
 *
 *  Created on: Jun 26, 2024
 *      Author: odemki
 */

#include "main.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

#include "DS3231/main_ds3231.h"
#include "DS3231/ds3231.h"

extern I2C_HandleTypeDef hi2c1;

// --------------------------------------------------------------------------------
void read_time(char * buff)
{
	time_i2c_write_single(DS3231_I2C_ADDRESS, 0x0E, 0x04);			// Turn Off square wave signal on SQW Pin DS3231
	time_i2c_write_single(DS3231_I2C_ADDRESS, 0x0F, 0x00);			// Turn Off square wave signal on 32K Pin DS3231

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
		strcat(buff, buf_1);
		memset(buf_1, 0, sizeof(buf_1));

		if(i > 0)
		{
			strcat(buff, ":");
		}
	}
	strcat(buff, " ");
}
// --------------------------------------------------------------------------------
