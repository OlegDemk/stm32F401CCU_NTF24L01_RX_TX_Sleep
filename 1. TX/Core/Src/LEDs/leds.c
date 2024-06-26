/*
 * leds.c
 *
 *  Created on: Jun 26, 2024
 *      Author: odemki
 */

#include "main.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

#include "LEDs/leds.h"

// --------------------------------------------------------------------------------
void led_test_blink(uint16_t times, uint16_t delay)
{
	for(int i = 0; i <= times; i++)
	{
		RED_LED_ON;
		GREEN_LED_ON;
		HAL_Delay(delay);
		RED_LED_OFF;
		GREEN_LED_OFF;
		HAL_Delay(delay);
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




