/*
 * main_bme280.c
 *
 *  Created on: Jun 24, 2024
 *      Author: odemki
 */

#include "main.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

#include "BME280/main_bme280.h"
#include "BME280/bme280.h"
#include "BME280/bme280_defs.h"



extern I2C_HandleTypeDef hi2c1;

struct bme280_dev dev;
struct bme280_data comp_data;
//	int8_t rslt;



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
void get_THP_bme280(char * buff)
{
	init_bme280_();

	float BME280_temperature = 0;
	float BME280_humidity = 0;
	float BME280_preasure = 0;

	if(bme280_get_sensor_data(BME280_ALL, &comp_data, &dev) == BME280_OK)
	{
		// Save data variables
		BME280_temperature = (float)comp_data.temperature;
		BME280_humidity = (float)comp_data.humidity;
		BME280_preasure = (float)comp_data.pressure;
	}
	else
	{
		error_blink_red_led();
	}

	if(put_device_to_sleep(&dev) != 0)
	{
		error_blink_red_led();
	}


	char buffer[10] = {0,};

	memset(buffer, 0, sizeof(buffer));
	sprintf(buffer, "T%.1f", BME280_temperature);
	strcat(buff, buffer);

	memset(buffer, 0, sizeof(buffer));
	sprintf(buffer, "H%.1f", BME280_humidity);
	strcat(buff, buffer);

}
