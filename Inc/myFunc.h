/*
 * myFunc.h
 *
 *  Created on: Oct 31, 2016
 *      Author: khockuba
 */

#ifndef MYFUNC_H_
#define MYFUNC_H_

#include "stm32f0xx_hal.h"
#include "main.h"

uint32_t usTicks;

#define LED0_ON HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);

#define LED0_OFF HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);

#define LED1_ON HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

#define LED1_OFF HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

#define SPI_SELECT HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

#define SPI_DESELECT HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

uint32_t getUs(void);
void delayUs(uint16_t *micros);

#endif /* MYFUNC_H_ */
