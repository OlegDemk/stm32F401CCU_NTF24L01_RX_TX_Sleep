/*
 * rtc_data_write_read.c
 *
 *  Created on: Jun 24, 2024
 *      Author: odemki
 */

#ifndef INC_RTC_DATA_RTC_DATA_WRITE_READ_C_
#define INC_RTC_DATA_RTC_DATA_WRITE_READ_C_

#define RTC_COUNTER_PACKET 1
#define RTC_COUNTER_SECOND 2							// For save periond
#define RTC_COUNTER_RETRANSMITED_PACKET 3
#define RTC_COUNTER_LOST_PACKET 4

void WriteBackupRegister(uint32_t data, uint8_t type);
uint32_t ReadBackupRegister(uint8_t type);

#endif /* INC_RTC_DATA_RTC_DATA_WRITE_READ_C_ */
