/**
 ******************************************************************************
 * @file    LSM6DSL_ACC_GYRO_driver.h
 * @author  MEMS Application Team
 * @modified Krzysztof Hockuba
 * @version V1.5
 * @date    17-May-2016
 * @brief   LSM6DSL header driver file
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LSM6DSL_ACC_GYRO_DRIVER__H
#define __LSM6DSL_ACC_GYRO_DRIVER__H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32f0xx_hal.h"

/* Exported types ------------------------------------------------------------*/

//these could change accordingly with the architecture

/* Exported common structure --------------------------------------------------------*/

typedef enum
{
  MEMS_SUCCESS        =   0x00,
  MEMS_ERROR        =   0x01
} status_t;

/* Exported macro ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/************** I2C Address *****************/

#define LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW   0xD4  // SAD[0] = 0
#define LSM6DSL_ACC_GYRO_I2C_ADDRESS_HIGH  0xD6  // SAD[0] = 1

/************** Who am I  *******************/

#define LSM6DSL_ACC_GYRO_WHO_AM_I         0x6A

/************** Device Register  *******************/

#define LSM6DSL_ACC_GYRO_FUNC_CFG_ACCESS    0X01

#define LSM6DSL_ACC_GYRO_SENSOR_SYNC_TIME   0X04
#define LSM6DSL_ACC_GYRO_SENSOR_RES_RATIO   0X05

#define LSM6DSL_ACC_GYRO_FIFO_CTRL1   0X06
#define LSM6DSL_ACC_GYRO_FIFO_CTRL2   0X07
#define LSM6DSL_ACC_GYRO_FIFO_CTRL3   0X08
#define LSM6DSL_ACC_GYRO_FIFO_CTRL4   0X09
#define LSM6DSL_ACC_GYRO_FIFO_CTRL5   0X0A

#define LSM6DSL_ACC_GYRO_DRDY_PULSE_CFG_G   0X0B
#define LSM6DSL_ACC_GYRO_INT1_CTRL    0X0D
#define LSM6DSL_ACC_GYRO_INT2_CTRL    0X0E
#define LSM6DSL_ACC_GYRO_WHO_AM_I_REG   0X0F
#define LSM6DSL_ACC_GYRO_CTRL1_XL   0X10
#define LSM6DSL_ACC_GYRO_CTRL2_G    0X11
#define LSM6DSL_ACC_GYRO_CTRL3_C    0X12
#define LSM6DSL_ACC_GYRO_CTRL4_C    0X13
#define LSM6DSL_ACC_GYRO_CTRL5_C    0X14
#define LSM6DSL_ACC_GYRO_CTRL6_G    0X15
#define LSM6DSL_ACC_GYRO_CTRL7_G    0X16
#define LSM6DSL_ACC_GYRO_CTRL8_XL   0X17
#define LSM6DSL_ACC_GYRO_CTRL9_XL   0X18
#define LSM6DSL_ACC_GYRO_CTRL10_C   0X19

#define LSM6DSL_ACC_GYRO_MASTER_CONFIG    0X1A
#define LSM6DSL_ACC_GYRO_WAKE_UP_SRC    0X1B
#define LSM6DSL_ACC_GYRO_TAP_SRC    0X1C
#define LSM6DSL_ACC_GYRO_D6D_SRC    0X1D
#define LSM6DSL_ACC_GYRO_STATUS_REG   0X1E

#define LSM6DSL_ACC_GYRO_OUT_TEMP_L   0X20
#define LSM6DSL_ACC_GYRO_OUT_TEMP_H   0X21
#define LSM6DSL_ACC_GYRO_OUTX_L_G   0X22
#define LSM6DSL_ACC_GYRO_OUTX_H_G   0X23
#define LSM6DSL_ACC_GYRO_OUTY_L_G   0X24
#define LSM6DSL_ACC_GYRO_OUTY_H_G   0X25
#define LSM6DSL_ACC_GYRO_OUTZ_L_G   0X26
#define LSM6DSL_ACC_GYRO_OUTZ_H_G   0X27
#define LSM6DSL_ACC_GYRO_OUTX_L_XL    0X28
#define LSM6DSL_ACC_GYRO_OUTX_H_XL    0X29
#define LSM6DSL_ACC_GYRO_OUTY_L_XL    0X2A
#define LSM6DSL_ACC_GYRO_OUTY_H_XL    0X2B
#define LSM6DSL_ACC_GYRO_OUTZ_L_XL    0X2C
#define LSM6DSL_ACC_GYRO_OUTZ_H_XL    0X2D
#define LSM6DSL_ACC_GYRO_SENSORHUB1_REG   0X2E
#define LSM6DSL_ACC_GYRO_SENSORHUB2_REG   0X2F
#define LSM6DSL_ACC_GYRO_SENSORHUB3_REG   0X30
#define LSM6DSL_ACC_GYRO_SENSORHUB4_REG   0X31
#define LSM6DSL_ACC_GYRO_SENSORHUB5_REG   0X32
#define LSM6DSL_ACC_GYRO_SENSORHUB6_REG   0X33
#define LSM6DSL_ACC_GYRO_SENSORHUB7_REG   0X34
#define LSM6DSL_ACC_GYRO_SENSORHUB8_REG   0X35
#define LSM6DSL_ACC_GYRO_SENSORHUB9_REG   0X36
#define LSM6DSL_ACC_GYRO_SENSORHUB10_REG    0X37
#define LSM6DSL_ACC_GYRO_SENSORHUB11_REG    0X38
#define LSM6DSL_ACC_GYRO_SENSORHUB12_REG    0X39
#define LSM6DSL_ACC_GYRO_FIFO_STATUS1   0X3A
#define LSM6DSL_ACC_GYRO_FIFO_STATUS2   0X3B
#define LSM6DSL_ACC_GYRO_FIFO_STATUS3   0X3C
#define LSM6DSL_ACC_GYRO_FIFO_STATUS4   0X3D
#define LSM6DSL_ACC_GYRO_FIFO_DATA_OUT_L    0X3E
#define LSM6DSL_ACC_GYRO_FIFO_DATA_OUT_H    0X3F
#define LSM6DSL_ACC_GYRO_TIMESTAMP0_REG   0X40
#define LSM6DSL_ACC_GYRO_TIMESTAMP1_REG   0X41
#define LSM6DSL_ACC_GYRO_TIMESTAMP2_REG   0X42

#define LSM6DSL_ACC_GYRO_TIMESTAMP_L    0X49
#define LSM6DSL_ACC_GYRO_TIMESTAMP_H    0X4A

#define LSM6DSL_ACC_GYRO_STEP_COUNTER_L   0X4B
#define LSM6DSL_ACC_GYRO_STEP_COUNTER_H   0X4C

#define LSM6DSL_ACC_GYRO_SENSORHUB13_REG    0X4D
#define LSM6DSL_ACC_GYRO_SENSORHUB14_REG    0X4E
#define LSM6DSL_ACC_GYRO_SENSORHUB15_REG    0X4F
#define LSM6DSL_ACC_GYRO_SENSORHUB16_REG    0X50
#define LSM6DSL_ACC_GYRO_SENSORHUB17_REG    0X51
#define LSM6DSL_ACC_GYRO_SENSORHUB18_REG    0X52

#define LSM6DSL_ACC_GYRO_FUNC_SRC   0X53
#define LSM6DSL_ACC_GYRO_TAP_CFG1   0X58
#define LSM6DSL_ACC_GYRO_TAP_THS_6D   0X59
#define LSM6DSL_ACC_GYRO_INT_DUR2   0X5A
#define LSM6DSL_ACC_GYRO_WAKE_UP_THS    0X5B
#define LSM6DSL_ACC_GYRO_WAKE_UP_DUR    0X5C
#define LSM6DSL_ACC_GYRO_FREE_FALL    0X5D
#define LSM6DSL_ACC_GYRO_MD1_CFG    0X5E
#define LSM6DSL_ACC_GYRO_MD2_CFG    0X5F

#define LSM6DSL_ACC_GYRO_OUT_MAG_RAW_X_L    0X66
#define LSM6DSL_ACC_GYRO_OUT_MAG_RAW_X_H    0X67
#define LSM6DSL_ACC_GYRO_OUT_MAG_RAW_Y_L    0X68
#define LSM6DSL_ACC_GYRO_OUT_MAG_RAW_Y_H    0X69
#define LSM6DSL_ACC_GYRO_OUT_MAG_RAW_Z_L    0X6A
#define LSM6DSL_ACC_GYRO_OUT_MAG_RAW_Z_H    0X6B

#define LSM6DSL_ACC_GYRO_X_OFS_USR    0X73
#define LSM6DSL_ACC_GYRO_Y_OFS_USR    0X74
#define LSM6DSL_ACC_GYRO_Z_OFS_USR    0X75

/************** Embedded functions register mapping  *******************/
#define LSM6DSL_ACC_GYRO_SLV0_ADD                     0x02
#define LSM6DSL_ACC_GYRO_SLV0_SUBADD                  0x03
#define LSM6DSL_ACC_GYRO_SLAVE0_CONFIG                0x04
#define LSM6DSL_ACC_GYRO_SLV1_ADD                     0x05
#define LSM6DSL_ACC_GYRO_SLV1_SUBADD                  0x06
#define LSM6DSL_ACC_GYRO_SLAVE1_CONFIG                0x07
#define LSM6DSL_ACC_GYRO_SLV2_ADD                     0x08
#define LSM6DSL_ACC_GYRO_SLV2_SUBADD                  0x09
#define LSM6DSL_ACC_GYRO_SLAVE2_CONFIG                0x0A
#define LSM6DSL_ACC_GYRO_SLV3_ADD                     0x0B
#define LSM6DSL_ACC_GYRO_SLV3_SUBADD                  0x0C
#define LSM6DSL_ACC_GYRO_SLAVE3_CONFIG                0x0D
#define LSM6DSL_ACC_GYRO_DATAWRITE_SRC_MODE_SUB_SLV0  0x0E
#define LSM6DSL_ACC_GYRO_CONFIG_PEDO_THS_MIN          0x0F

#define LSM6DSL_ACC_GYRO_SM_STEP_THS                  0x13
#define LSM6DSL_ACC_GYRO_PEDO_DEB_REG                0x14
#define LSM6DSL_ACC_GYRO_STEP_COUNT_DELTA            0x15

#define LSM6DSL_ACC_GYRO_MAG_SI_XX                    0x24
#define LSM6DSL_ACC_GYRO_MAG_SI_XY                    0x25
#define LSM6DSL_ACC_GYRO_MAG_SI_XZ                    0x26
#define LSM6DSL_ACC_GYRO_MAG_SI_YX                    0x27
#define LSM6DSL_ACC_GYRO_MAG_SI_YY                    0x28
#define LSM6DSL_ACC_GYRO_MAG_SI_YZ                    0x29
#define LSM6DSL_ACC_GYRO_MAG_SI_ZX                    0x2A
#define LSM6DSL_ACC_GYRO_MAG_SI_ZY                    0x2B
#define LSM6DSL_ACC_GYRO_MAG_SI_ZZ                    0x2C
#define LSM6DSL_ACC_GYRO_MAG_OFFX_L                   0x2D
#define LSM6DSL_ACC_GYRO_MAG_OFFX_H                   0x2E
#define LSM6DSL_ACC_GYRO_MAG_OFFY_L                   0x2F
#define LSM6DSL_ACC_GYRO_MAG_OFFY_H                   0x30
#define LSM6DSL_ACC_GYRO_MAG_OFFZ_L                   0x31
#define LSM6DSL_ACC_GYRO_MAG_OFFZ_H                   0x32

//Public Functions

status_t LSM6DSL_Init(SPI_HandleTypeDef *spi);
status_t LSM6DSL_AccRaw(int16_t *acc);
status_t LSM6DSL_GyroRaw(int16_t *gyro);
status_t LSM6DSL_TempRaw(int16_t *temperature);
status_t LSM6DSL_Acc(float *acc);
status_t LSM6DSL_Gyro(float *acc);
status_t LSM6DSL_Temp(float *temperature);

status_t LSM6DSL_ACC_GYRO_ReadReg(uint8_t reg, uint8_t* Data, uint16_t len);

#endif
