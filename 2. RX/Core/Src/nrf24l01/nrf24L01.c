/*
 * nrf24L01.c
 *
 *  Created on: Apr 6, 2023
 *      Author: odemki
 */


#include "main.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
//#include "cmsis_os.h"

#include "nrf24L01/nrf24L01.h"
//#include "cmsis_os.h"
//#include "queue.h"

#include "stdbool.h"

#define TX_ADR_WIDTH 3
#define TX_PLOAD_WIDTH 32

extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart1;
//extern osMessageQueueId_t DATAQueueHandle;

uint8_t pipe_num = 0;

uint8_t RX_BUF[TX_PLOAD_WIDTH] = {0};
uint8_t TX_ADDRESS_0[TX_ADR_WIDTH] = {0xb2,0xb4,0x01};
uint8_t TX_ADDRESS_1[TX_ADR_WIDTH] = {0xb7,0xb5,0xa1};
#define TX_ADDRESS_2 0xb6
//uint8_t TX_ADDRESS_2[TX_ADR_WIDTH] = {0xb6,0xb5,0xa1};
//uint8_t TX_ADDRESS_3[TX_ADR_WIDTH] = {0xb5,0xb5,0xa1};
//uint8_t TX_ADDRESS_4[TX_ADR_WIDTH] = {0xb4,0xb5,0xa1};
//uint8_t TX_ADDRESS_5[TX_ADR_WIDTH] = {0xb3,0xb5,0xa1};


uint8_t NRF24_ReadReg(uint8_t addr);
static void NRF24_ToggleFeatures(void);
static void NRF24_WriteReg(uint8_t addr, uint8_t dt);
static void NRF24_FlushRX(void);
static void NRF24_FlushTX(void);
void NRF24L01_RX_Mode(void);
void NRF24_Write_Buf(uint8_t addr,uint8_t *pBuf,uint8_t bytes);
void print_Data_Ower_uart(uint8_t *RX_BUF, uint8_t *pipe);

volatile uint8_t rx_flag = 0;

// -------------------------------------------------------------------------------------
__STATIC_INLINE void DelayMicro(__IO uint32_t micros)
{
	micros *= (SystemCoreClock / 1000000) / 7;   // 9
	/* Wait till done */
	while (micros--) ;
}
// -------------------------------------------------------------------------------------
void NRF24_Sleep_mode(void)
{
	CE_RESET;
	HAL_Delay(5);
	NRF24_WriteReg(CONFIG, 0x00);
	HAL_Delay(5);


}
// -------------------------------------------------------------------------------------
void NRF24_init_RX(uint8_t zero_pipe, uint8_t first_pipe, uint8_t second_pipe,
		uint8_t third_pipe,	uint8_t fourth_pipe, uint8_t fifth_pipe,
		uint8_t chanel, uint8_t data_rate, uint8_t output_tx_power)
{
	if(chanel < 0 || chanel > 127)
	{
		chanel = 10;
	}

	 CE_RESET;
	 // DelayMicro(5000);
	 //osDelay(5);
	  HAL_Delay(5);
	 NRF24_WriteReg(CONFIG, 0x0A); 			// Set PWR_UP bit, enable CRC(1 byte) &Prim_RX:0 (Transmitter)
	 // DelayMicro(5000);
	  HAL_Delay(5);
//	 osDelay(5);

	 //  Define pipes
	 	 uint8_t rx_pipes = (zero_pipe ) | (first_pipe << 1) | (second_pipe << 2) |
	 			(third_pipe << 3) | (fourth_pipe << 4) | (fifth_pipe << 5);

	 NRF24_WriteReg(EN_AA, 0x07); 				// Enable pipe0, pipe1 and pipe2
	 NRF24_WriteReg(EN_RXADDR, 0x07); 			// Enable pipe0, pipe1 and pipe2				// включає канал
	 NRF24_WriteReg(SETUP_AW, 0x01); 			// Setup address width=3 bytes
	 NRF24_WriteReg(SETUP_RETR, 0x5F);			// 1500us, 15 retrans

	 NRF24_ToggleFeatures();

	 NRF24_WriteReg(FEATURE, 0);				// динамічна довжина пакетів
	 NRF24_WriteReg(DYNPD, 0);					// Вказує які канали використовують динамічну зміну пакетів
	 NRF24_WriteReg(STATUSS, 0x70);				// Reset flags for IRQ when data arrived
	 NRF24_WriteReg(RF_CH, 76); 				// 2476 MHz   2400 + 76

	 NRF24_WriteReg(RF_SETUP, 0x06);  		//TX_PWR:0dBm, Datarate:1Mbps
//	 NRF24_WriteReg(RF_SETUP, data_rate|output_tx_power);  		// TX_PWR:0dBm, Datarate: 250kbp	- New version

	 NRF24_Write_Buf(TX_ADDR, TX_ADDRESS_0, TX_ADR_WIDTH);

	 NRF24_Write_Buf(RX_ADDR_P0, TX_ADDRESS_0, TX_ADR_WIDTH);
	 NRF24_Write_Buf(RX_ADDR_P1, TX_ADDRESS_1, TX_ADR_WIDTH);
	 NRF24_WriteReg(RX_ADDR_P2, TX_ADDRESS_2);

	 NRF24_WriteReg(RX_PW_P0, TX_PLOAD_WIDTH); 	//Number of bytes in RX payload in data pipe 0
	 NRF24_WriteReg(RX_PW_P1, TX_PLOAD_WIDTH); 	//Number of bytes in RX payload in data pipe 1
	 NRF24_WriteReg(RX_PW_P2, TX_PLOAD_WIDTH); 	//Number of bytes in RX payload in data pipe 2

	 NRF24L01_RX_Mode();
	 LED_OFF;
}
// -------------------------------------------------------------------------------------
void NRF24_init_TX(uint8_t pipe, uint8_t chanel, uint8_t retrans_delay, uint8_t retransmit_attempt, uint8_t data_rate, uint8_t output_tx_power)
{
	if(chanel < 0 || chanel > 127)
	{
		chanel = 10;
	}

	CE_RESET;
	// DelayMicro(5000);
//	osDelay(5);
	 HAL_Delay(5);
	NRF24_WriteReg(CONFIG, 0x0A); 			// Set PWR_UP bit, enable CRC(1 byte) &Prim_RX:0 (Transmitter)
	// DelayMicro(5000);
	 HAL_Delay(5);
//	osDelay(5);

	NRF24_WriteReg(EN_AA, 0x01); 			// Enable pipe 0
	NRF24_WriteReg(EN_RXADDR, 0x01); 		// Enable Pipe 0				// включає канал

	NRF24_WriteReg(SETUP_AW, 0x01); 		// Setup address width=3 bytes

//	uint8_t SETUP_RETR_data = (retrans_delay << 4) | (retransmit_attempt);
//	NRF24_WriteReg(SETUP_RETR, SETUP_RETR_data);		// 1500us, 15 retrans           0x7F
	NRF24_WriteReg(SETUP_RETR, 0x5F);

	NRF24_ToggleFeatures();

	NRF24_WriteReg(FEATURE, 0);
	NRF24_WriteReg(DYNPD, 0);
	NRF24_WriteReg(STATUSS, 0x70);			// Reset flags for IRQ
	NRF24_WriteReg(RF_CH, 76);
//	NRF24_WriteReg(RF_CH, chanel); 			//  2400 + chanel MHz


	NRF24_WriteReg(RF_SETUP, data_rate|output_tx_power);  		// TX_PWR:0dBm, Datarate: 250kbp	- New version

	if(pipe == 0)
	{
		NRF24_Write_Buf(TX_ADDR, TX_ADDRESS_0, TX_ADR_WIDTH);											// Write TX address
		NRF24_Write_Buf(RX_ADDR_P0, TX_ADDRESS_0, TX_ADR_WIDTH);											// Write RX address Pipe 0
		NRF24_WriteReg(RX_PW_P0, TX_PLOAD_WIDTH);	 													// Number of bytes in RX payload in data pipe 0
	}
//	if(pipe == 1)
//	{
//		NRF24_Write_Buf(TX_ADDR, TX_ADDRESS_1, TX_ADR_WIDTH);											// Write TX address
//		NRF24_Write_Buf(RX_ADDR_P0, TX_ADDRESS_1, TX_ADR_WIDTH);											// Write RX address Pipe 0
//		NRF24_WriteReg(RX_PW_P0, TX_PLOAD_WIDTH);	 													// Number of bytes in RX payload in data pipe 0
//	}
//	if(pipe == 2)
//	{
//		NRF24_Write_Buf(TX_ADDR, TX_ADDRESS_2, TX_ADR_WIDTH);											// Write TX address
//		NRF24_Write_Buf(RX_ADDR_P0, TX_ADDRESS_2, TX_ADR_WIDTH);											// Write RX address Pipe 0
//		NRF24_WriteReg(RX_PW_P0, TX_PLOAD_WIDTH);	 													// Number of bytes in RX payload in data pipe 0
//	}
//	if(pipe == 3)
//	{
//		NRF24_Write_Buf(TX_ADDR, TX_ADDRESS_3, TX_ADR_WIDTH);											// Write TX address
//		NRF24_Write_Buf(RX_ADDR_P0, TX_ADDRESS_3, TX_ADR_WIDTH);											// Write RX address Pipe 0
//		NRF24_WriteReg(RX_PW_P0, TX_PLOAD_WIDTH);	 													// Number of bytes in RX payload in data pipe 0
//	}
//
//	if(pipe == 4)
//	{
//		NRF24_Write_Buf(TX_ADDR, TX_ADDRESS_4, TX_ADR_WIDTH);											// Write TX address
//		NRF24_Write_Buf(RX_ADDR_P0, TX_ADDRESS_4, TX_ADR_WIDTH);											// Write RX address Pipe 0
//		NRF24_WriteReg(RX_PW_P0, TX_PLOAD_WIDTH);	 													// Number of bytes in RX payload in data pipe 0
//	}
//
//	if(pipe == 5)
//	{
//		NRF24_Write_Buf(TX_ADDR, TX_ADDRESS_5, TX_ADR_WIDTH);											// Write TX address
//		NRF24_Write_Buf(RX_ADDR_P0, TX_ADDRESS_5, TX_ADR_WIDTH);											// Write RX address Pipe 0
//		NRF24_WriteReg(RX_PW_P0, TX_PLOAD_WIDTH);	 													// Number of bytes in RX payload in data pipe 0
//	}


	 NRF24L01_RX_Mode();
	 LED_OFF;
}
// -------------------------------------------------------------------------------------
uint8_t NRF24_ReadReg(uint8_t addr)
{
	uint8_t dt=0, cmd;
	CS_ON;

	HAL_SPI_TransmitReceive(&hspi2, &addr, &dt, 1, 1000);

	if (addr != STATUSS)		//если адрес равен адрес регистра статус то и возварщаем его состояние
	{
		cmd = 0xFF;
		HAL_SPI_TransmitReceive(&hspi2, &cmd, &dt, 1, 1000);
	}

	CS_OFF;

	return dt;
}
// -------------------------------------------------------------------------------------
void NRF24_WriteReg(uint8_t addr, uint8_t dt)
{
	addr |= W_REGISTER;//включим бит записи в адрес

	CS_ON;

	HAL_SPI_Transmit(&hspi2,&addr,1,1000);	//отправим адрес в шину
	HAL_SPI_Transmit(&hspi2,&dt,1,1000);	//отправим данные в шину

	CS_OFF;
}
// -------------------------------------------------------------------------------------
void NRF24_ToggleFeatures(void)
{
	uint8_t dt[1] = {ACTIVATE};

	CS_ON;
	HAL_SPI_Transmit(&hspi2,dt,1,1000);

	DelayMicro(1);

	dt[0] = 0x73;
	HAL_SPI_Transmit(&hspi2,dt,1,1000);
	CS_OFF;
}
// -------------------------------------------------------------------------------------
void NRF24_Read_Buf(uint8_t addr,uint8_t *pBuf,uint8_t bytes)
{
	CS_ON;
	HAL_SPI_Transmit(&hspi2,&addr,1,1000);				//отправим адрес в шину

	HAL_SPI_Receive(&hspi2,pBuf,bytes,1000);			//отправим данные в буфер

	CS_OFF;
}
// -------------------------------------------------------------------------------------
void NRF24_Write_Buf(uint8_t addr,uint8_t *pBuf,uint8_t bytes)
{
	addr |= W_REGISTER;									//включим бит записи в адрес

	CS_ON;
	HAL_SPI_Transmit(&hspi2,&addr,1,1000);				//отправим адрес в шину

	DelayMicro(1);

	HAL_SPI_Transmit(&hspi2,pBuf,bytes,1000);			//отправим данные в буфер

	CS_OFF;
}
// -------------------------------------------------------------------------------------
static void NRF24_FlushRX(void)
{
	uint8_t dt[1] = {FLUSH_RX};

	CS_ON;
	HAL_SPI_Transmit(&hspi2,dt,1,1000);
	DelayMicro(1);
	CS_OFF;
}
// -------------------------------------------------------------------------------------
static void NRF24_FlushTX(void)
{
	uint8_t dt[1] = {FLUSH_TX};

	CS_ON;
	HAL_SPI_Transmit(&hspi2,dt,1,1000);
	DelayMicro(1);
	CS_OFF;
}
// -------------------------------------------------------------------------------------
void NRF24L01_RX_Mode(void)
{
	uint8_t regval=0x00;
	regval = NRF24_ReadReg(CONFIG);

	regval |= (1<<PWR_UP)|(1<<PRIM_RX);

	NRF24_WriteReg(CONFIG, regval);
	CE_SET;

	DelayMicro(150);

	// Flush buffers
	NRF24_FlushRX();
	NRF24_FlushTX();
}
// -------------------------------------------------------------------------------------
void testReadWriteSetingd(void)
{
	uint8_t buf1[40] = {0};
	uint8_t str1[40] = {0};
	uint8_t dt_reg=0;

	dt_reg = NRF24_ReadReg(CONFIG);
	sprintf(str1,"CONFIG: 0x%02X\n\r",dt_reg);
	HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);

	dt_reg = NRF24_ReadReg(EN_AA);
	sprintf(str1,"EN_AA: 0x%02X\n\r",dt_reg);
	HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);

	dt_reg = NRF24_ReadReg(EN_RXADDR);
	sprintf(str1,"EN_RXADDR: 0x%02X\n\r",dt_reg);
	HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);

	dt_reg = NRF24_ReadReg(STATUSS);
	sprintf(str1,"STATUS: 0x%02X\n\r",dt_reg);
	HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);

	dt_reg = NRF24_ReadReg(RF_SETUP);
	sprintf(str1,"RF_SETUP: 0x%02X\n\r",dt_reg);
	HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);

    NRF24_Read_Buf(TX_ADDR,buf1,3);
	sprintf(str1,"TX_ADDR: 0x%02X, 0x%02X, 0x%02X\n\r",buf1[0],buf1[1],buf1[2]);
	HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);

	NRF24_Read_Buf(RX_ADDR_P0,buf1,3);
	sprintf(str1,"RX_ADDR: 0x%02X, 0x%02X, 0x%02X\n\r",buf1[0],buf1[1],buf1[2]);
	HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
}
// -------------------------------------------------------------------------------------
void testDelay(void)
{
	//HAL_GPIO_TogglePin(GPIOC, LED_Pin);
	DelayMicro(1000);
}
// -------------------------------------------------------------------------------------
void NRF24L01_TX_Mode(void)
{
	NRF24_Write_Buf(TX_ADDR, TX_ADDRESS_0, TX_ADR_WIDTH);		// ????
	CE_RESET;

	// Flush buffers
	NRF24_FlushRX();
	NRF24_FlushTX();
}
// -------------------------------------------------------------------------------------
void NRF24_Transmit(uint8_t addr,uint8_t *pBuf,uint8_t bytes)
{
	CE_RESET;
	CS_ON;

	HAL_SPI_Transmit(&hspi2,&addr,1,1000);			// Send address in buss
	DelayMicro(1);
	HAL_SPI_Transmit(&hspi2,pBuf,bytes,1000);		// Send data in buss

	CS_OFF;
	CE_SET;
}
// -------------------------------------------------------------------------------------
uint8_t NRF24L01_Send(uint8_t *pBuf)
{
	uint8_t regval = 0;

	return regval;
}
// -------------------------------------------------------------------------------------
void NRF24L01_Transmit(uint8_t *pipe_address, char *data[])
{
	NRF24L01_Send(data);


}
// ------------------------------------------------------------------------------------
void led_test(void)
{
	 // LED_ON;
	 LED_OFF;
}
// -------------------------------------------------------------------------------------
void NRF24L01_Receive(void)
{
	if(rx_flag == 1)
	{
		print_Data_Ower_uart(RX_BUF, 1);

		rx_flag = 0;
		LED_OFF;
	}

}
// -------------------------------------------------------------------------------------
void IRQ_Callback(void)
{
	uint8_t status=0x01;
	uint8_t pipe;

	LED_ON;

	DelayMicro(10);

	status = NRF24_ReadReg(STATUSS);

	if(status & 0x40)											// If data ready in FIFO buff
	{
		pipe = (status>>1)&0x07;

		pipe_num = pipe;

		NRF24_Read_Buf(RD_RX_PLOAD, RX_BUF, TX_PLOAD_WIDTH);		// Copy into buffer
		NRF24_WriteReg(STATUSS, 0x40);

		rx_flag = 1;
	}
	LED_OFF;
}
// -------------------------------------------------------------------------------------
void print_Data_Ower_uart(uint8_t *RX_BUF, uint8_t *pipe)
{
	char buff_uart[90] = {0};

	sprintf(buff_uart, "Data from PIPE %d: %s\n\r", pipe_num, RX_BUF);
	HAL_UART_Transmit(&huart1, (char*)buff_uart, sizeof(buff_uart), 1000);
}
// -------------------------------------------------------------------------------------
















