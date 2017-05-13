/*
 * nt.c
 *
 *  Created on: Oct 30, 2016
 *      Author: khockuba
 */

#include "stm32f0xx_hal.h"
#include "nt.h"
#include "main.h"
#include "usart.h"
#include "string.h"
#include "stdlib.h"

#define NTBUS_GETIMU_DATALEN              (sizeof(tNTBusGetImuData))

uint8_t txBuffer[20];
volatile uint8_t txcomplete = 1;
ntImuConf *imu;
tNTBusGetImuData data;

void put_c(uint8_t letter) {
	while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) == 0);
	huart1.Instance->TDR = letter & 0xFF;
	while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) == 0);
}

void enableTX(void) {
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF1_USART1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	SET_BIT(huart1.Instance->CR1, USART_CR1_TE);
}

void disableTX(void) {
	CLEAR_BIT(huart1.Instance->CR1, USART_CR1_TE);
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9);
}

uint8_t buffer[32];

void ntSendIMU(void *imu) {
	uint8_t crc = 0;
	while(huart1.gState != HAL_UART_STATE_READY);
	memcpy(buffer, imu, NTBUS_GETIMU_DATALEN);
	enableTX();
	for(uint8_t i=0; i<NTBUS_GETIMU_DATALEN; i++ ) {
		crc ^= buffer[i];
	}
	buffer[NTBUS_GETIMU_DATALEN] = crc;
	HAL_UART_Transmit_DMA(&huart1, buffer, NTBUS_GETIMU_DATALEN + 1);
}

static void ntStatus(void) {
	while(huart1.gState != HAL_UART_STATE_READY);
	buffer[0] = NTBUS_IMU_STATUS_IMU_PRESENT;
	buffer[1] = 0x1;
	buffer[2] = NTBUS_IMU_STATUS_IMU_PRESENT ^ 0x1;
	enableTX();
	HAL_UART_Transmit_DMA(&huart1, buffer, 3);
}

static void ntSendVersion(void) {
	while(huart1.gState != HAL_UART_STATE_READY);
	memset(buffer, 0x00, 17);
	sprintf((char*) buffer, "v0.23");
	uint8_t crc = 0x00;
	size_t size = strlen((char*) buffer);
	for (uint8_t i = 0; i < size; i++) {
		crc ^= buffer[i];
	}
	buffer[16] = crc;
	enableTX();
	HAL_UART_Transmit_DMA(&huart1, buffer, 17);
}

static void ntSendBoard(void) {
	while(huart1.gState != HAL_UART_STATE_READY);
	memset(buffer, 0x00, 17);
	sprintf((char*) buffer, "F042K6");
	uint8_t crc = 0x00;
	size_t size = strlen((char*) buffer);
	for (uint8_t i = 0; i < size; i++) {
		crc ^= buffer[i];
	}
	buffer[16] = crc;
	enableTX();
	HAL_UART_Transmit_DMA(&huart1, buffer, 17);
}

static void ntSendConf(void) {
	while(huart1.gState != HAL_UART_STATE_READY);
	buffer[0] = NTBUS_IMU_CONFIG_FAST | NTBUS_IMU_CONFIG_MPU6000;
	buffer[1] = 0;
	buffer[2] = NTBUS_IMU_CONFIG_FAST | NTBUS_IMU_CONFIG_MPU6000;
	enableTX();
	HAL_UART_Transmit_DMA(&huart1, buffer, 3);
}

void ntInit(ntImuConf *conf) {
	imu = conf;
	conf->imuData = &data;
	if (HAL_GPIO_ReadPin(MODULE_SET_GPIO_Port, MODULE_SET_Pin) == GPIO_PIN_SET) conf->myID = NTBUS_ID_IMU1; else conf->myID = NTBUS_ID_IMU2;
	memset(&data, 0x00, sizeof(tNTBusGetImuData));
	conf->imuData->ImuStatus = NTBUS_IMU_IMUSTATUS_BASE | NTBUS_IMU_IMUSTATUS_ACCDATA_OK | NTBUS_IMU_IMUSTATUS_GYRODATA_OK;
	disableTX();
}

void ntParseCommand(uint8_t command) {
	switch(imu->frame.state) {
		case WAITINGFORSTX:
			//ID ADDRESS
			imu->frame.id = (command & NTBUS_IDMASK);
			//SHORTCOMMAND
			imu->frame.command = (command & NTBUS_SFMASK);
			if (!(command & NTBUS_STX)) return; //quit if we dont have 1 as 7th bit
			if (imu->myID != imu->frame.id) return;
			switch (imu->frame.command) {
				case NTBUS_GET:
					ntSendIMU(&data);
					break;
				case NTBUS_CMD:
					imu->frame.state = WAITINGFORGETCOMMAND;
					break;
				default: ;
			}
			break;
		case WAITINGFORGETCOMMAND:
			switch(command) {
				case NTBUS_CMD_GETSTATUS:
					ntStatus();
					break;
				case NTBUS_CMD_GETVERSIONSTR:
					ntSendVersion();
					break;
				case NTBUS_CMD_GETBOARDSTR:
					ntSendBoard();
					break;
				case NTBUS_CMD_GETCONFIGURATION:
					ntSendConf();
					break;
				default: ;
			}
			imu->frame.state = WAITINGFORSTX;
			break;
			default: ;
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart1) {
		disableTX();
	}
}
