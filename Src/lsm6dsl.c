/**
 ******************************************************************************
 * @file    LSM6DSL_ACC_GYRO_driver.c
 * @author  MEMS Application Team
 * @modified Krzysztof Hockuba
 * @version V1.5
 * @date    17-May-2016
 * @brief   LSM6DSL driver file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "lsm6dsl.h"

#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include "myFunc.h"
/* Imported function prototypes ----------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define MEMS_SUCCESS 0
#define MEMS_ERROR 1

#define DELAY(x) HAL_Delay(x)
#define DELAY_US(x) HAL_Delay(x)

#ifndef LSM6_CS_GPIO_Port
#define LSM6_CS_GPIO_Port SPI_CS_GPIO_Port
#endif

#ifndef LSM6_CS_Pin
#define LSM6_CS_Pin SPI_CS_Pin
#endif

#define BIT_SET(a,b) ((a) |= (1<<b))
#define BIT_CLEAR(a,b) ((a) &= ~(1<<b))
#define BIT_FLIP(a,b) ((a) ^= (1<<b))
#define BIT_CHECK(a,b) ((a) & (1<<b))

#define BITMASK_SET(a,b) ((a) |= (b))
#define BITMASK_CLEAR(a,b) ((a) &= ~(b))
#define BITMASK_FLIP(a,b) ((a) ^= (b))
#define BITMASK_CHECK(a,b) ((a) & (b))

/* Private macro -------------------------------------------------------------*/
#define LSM6_ENABLE HAL_GPIO_WritePin(LSM6_CS_GPIO_Port, LSM6_CS_Pin, GPIO_PIN_RESET);
#define LSM6_DISABLE HAL_GPIO_WritePin(LSM6_CS_GPIO_Port, LSM6_CS_Pin, GPIO_PIN_SET);
/* Private variables ---------------------------------------------------------*/
static SPI_HandleTypeDef *LSM6_SPI;
/* Private functions ---------------------------------------------------------*/

/* Exported functions ---------------------------------------------------------*/

/************** Generic Function  *******************/

/*******************************************************************************
* Function Name   : LSM6DSL_ACC_GYRO_ReadReg
* Description   : Generic Reading function. It must be fullfilled with either
*         : I2C or SPI reading functions
* Input       : Register Address, length of buffer
* Output      : Data REad
* Return      : None
*******************************************************************************/
status_t LSM6DSL_ACC_GYRO_ReadReg(uint8_t reg, uint8_t* Data, uint16_t len)
{
  status_t stat;
  LSM6_ENABLE;
  reg = reg | 0x80;
  stat = HAL_SPI_Transmit(LSM6_SPI, &reg, 1, HAL_MAX_DELAY);
  stat = HAL_SPI_Receive(LSM6_SPI, Data, len, HAL_MAX_DELAY);
  LSM6_DISABLE;
  if (stat) return MEMS_ERROR;
  else return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name   : LSM6DSL_ACC_GYRO_WriteReg
* Description   : Generic Writing function. It must be fullfilled with either
*         : I2C or SPI writing function
* Input       : Register Address, Data to be written, length of buffer
* Output      : None
* Return      : None
*******************************************************************************/
static status_t LSM6DSL_ACC_GYRO_WriteReg(uint8_t Reg, uint8_t *Data, uint16_t len)
{
  status_t stat;
  DELAY_US(10);
  LSM6_ENABLE;
  stat = HAL_SPI_Transmit(LSM6_SPI, &Reg, 1, HAL_MAX_DELAY);
  stat = HAL_SPI_Transmit(LSM6_SPI, Data, len, HAL_MAX_DELAY);
  LSM6_DISABLE;
  DELAY_US(10);
  if (stat) return MEMS_ERROR;
  else return MEMS_SUCCESS;
}

static status_t LSM6DSL_ACC_GYRO_WriteVerify(uint8_t Reg, uint8_t *Data, uint16_t len) {
	uint8_t temp;
	LSM6DSL_ACC_GYRO_WriteReg(Reg, Data, len);
	LSM6DSL_ACC_GYRO_ReadReg(Reg, &temp, len);
	if (*Data == temp) return MEMS_SUCCESS;
	return MEMS_ERROR;
}

status_t LSM6DSL_Init(SPI_HandleTypeDef *spi) {
	LSM6_SPI = spi;
	uint8_t temp;
	uint8_t tries = 10;
	while (tries--) {
		LSM6DSL_ACC_GYRO_ReadReg(LSM6DSL_ACC_GYRO_WHO_AM_I_REG, &temp, 1);
		if (temp == 0x69) break;
		HAL_Delay(50);
		if (temp != LSM6DSL_ACC_GYRO_WHO_AM_I && !tries) return MEMS_ERROR;
	}
//	LSM6DSL_ACC_GYRO_WriteReg(LSM6DSL_ACC_GYRO_CTRL3_C, (uint8_t*) 0x01, 1);
//	DELAY(100); // Software Reset
	LSM6DSL_ACC_GYRO_ReadReg(LSM6DSL_ACC_GYRO_CTRL3_C, &temp, 1);
	BIT_SET(temp, 2);
	LSM6DSL_ACC_GYRO_WriteVerify(LSM6DSL_ACC_GYRO_CTRL3_C, (uint8_t*) &temp, 1);
	temp = (0x05 << 4) | (0x02 << 2);
	if (LSM6DSL_ACC_GYRO_WriteVerify(LSM6DSL_ACC_GYRO_CTRL1_XL, (uint8_t *) &temp, 1)) return MEMS_ERROR; //208Hz, 16G
	temp = (0x05 << 4) | (0x02 << 2);
	if (LSM6DSL_ACC_GYRO_WriteVerify(LSM6DSL_ACC_GYRO_CTRL2_G, (uint8_t*) &temp, 1)) return MEMS_ERROR; //208Hz, 2000dps
	temp = (0x01 << 2);
	if (LSM6DSL_ACC_GYRO_WriteVerify(LSM6DSL_ACC_GYRO_CTRL4_C, (uint8_t*) &temp, 1)) return MEMS_ERROR; //Disable I2C
	temp = 0x01 | (0x01 << 1);
	if (LSM6DSL_ACC_GYRO_WriteVerify(LSM6DSL_ACC_GYRO_INT2_CTRL, (uint8_t*) &temp, 1)) return MEMS_ERROR; //Enable DRDY for Acc and Gyro on INT2
	return MEMS_SUCCESS;
}

status_t LSM6DSL_AccRaw(int16_t *acc) {
	uint8_t temp[6];
	LSM6DSL_ACC_GYRO_ReadReg(LSM6DSL_ACC_GYRO_OUTX_L_XL, temp, 6);
	acc[0] = (int16_t) temp[0] | ((int16_t) temp[1] << 8);
	acc[1] = (int16_t) temp[2] | ((int16_t) temp[3] << 8);
	acc[2] = (int16_t) temp[4] | ((int16_t) temp[5] << 8);
	return MEMS_SUCCESS;
}

status_t LSM6DSL_GyroRaw(int16_t *gyro) {
	uint8_t temp[6];
	LSM6DSL_ACC_GYRO_ReadReg(LSM6DSL_ACC_GYRO_OUTX_L_G, temp, 6);
	gyro[0] = (int16_t) temp[0] | ((int16_t) temp[1] << 8);
	gyro[1] = (int16_t) temp[2] | ((int16_t) temp[3] << 8);
	gyro[2] = (int16_t) temp[4] | ((int16_t) temp[5] << 8);
	return MEMS_SUCCESS;
}

status_t LSM6DSL_Acc(float *acc) {
	int16_t raw[3];
	LSM6DSL_AccRaw(raw);
	acc[0] = (float) raw[0] / (32767 / 16);
	acc[1] = (float) raw[1] / (32767 / 16);
	acc[2] = (float) raw[2] / (32767 / 16);
	return MEMS_SUCCESS;
}

status_t LSM6DSL_Gyro(float *acc) {
	int16_t raw[3];
	LSM6DSL_GyroRaw(raw);
	acc[0] = (float) raw[0] / (32767 / 2000);
	acc[1] = (float) raw[1] / (32767 / 2000);
	acc[2] = (float) raw[2] / (32767 / 2000);
	return MEMS_SUCCESS;
}

status_t LSM6DSL_TempRaw(int16_t *temperature) {
	uint8_t temp[2];
	LSM6DSL_ACC_GYRO_ReadReg(LSM6DSL_ACC_GYRO_OUT_TEMP_L, temp, 2);
	*temperature = (int16_t) temp[0] | ((int16_t) temp[1] << 8);
	return MEMS_SUCCESS;
}

status_t LSM6DSL_Temp(float *temperature) {
	int16_t temp;
	LSM6DSL_TempRaw(&temp);
	*temperature = (float) temp / 256;
	return MEMS_SUCCESS;
}

status_t LSM6DSL_EnableTilt(void) {
	uint8_t temp;
	//Enable Interupt on INT2
	LSM6DSL_ACC_GYRO_ReadReg(LSM6DSL_ACC_GYRO_MD2_CFG, &temp, 1);
	BIT_SET(temp, 1);
	LSM6DSL_ACC_GYRO_WriteVerify(LSM6DSL_ACC_GYRO_MD2_CFG, &temp, 1);
	LSM6DSL_ACC_GYRO_ReadReg(LSM6DSL_ACC_GYRO_CTRL10_C, &temp, 1);
	BIT_SET(temp, 2);
	BIT_SET(temp, 3);
	return LSM6DSL_ACC_GYRO_WriteVerify(LSM6DSL_ACC_GYRO_CTRL10_C, &temp, 1);
}

status_t LSM6DSL_EnableInactivity(void) {
	uint8_t temp;
	//Enable Interupt on INT2
	LSM6DSL_ACC_GYRO_ReadReg(LSM6DSL_ACC_GYRO_MD2_CFG, &temp, 1);
	BIT_SET(temp, 5);
	LSM6DSL_ACC_GYRO_WriteVerify(LSM6DSL_ACC_GYRO_MD2_CFG, &temp, 1);
	LSM6DSL_ACC_GYRO_ReadReg(LSM6DSL_ACC_GYRO_TAP_CFG1, &temp, 1);
	temp |= 0x02 << 5;
	LSM6DSL_ACC_GYRO_WriteVerify(LSM6DSL_ACC_GYRO_TAP_CFG1, &temp, 1);
	//Leave 16ODR as is.
	temp = 8;
	return LSM6DSL_ACC_GYRO_WriteVerify(LSM6DSL_ACC_GYRO_WAKE_UP_THS, &temp, 1);
}
