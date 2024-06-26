/*
 * main_bme280.h
 *
 *  Created on: Jun 24, 2024
 *      Author: odemki
 */

#ifndef INC_BME280_MAIN_BME280_H_
#define INC_BME280_MAIN_BME280_H_

int8_t init_bme280(void);
void bme280_measure(void);
void get_THP_bme280(char * buff);
void init_bme280_(void);

#endif /* INC_BME280_MAIN_BME280_H_ */
