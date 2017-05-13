#ifndef __NT_H
#define __NT_H

//NT PROTOCOL RELATED DEFINES
#include  "stm32f0xx_hal.h"

//Possible NT Modules
#define NTBUS_ID_ALLMODULES               0
#define NTBUS_ID_IMU1                     1
#define NTBUS_ID_IMU2                     2
#define NTBUS_ID_MOTORALL                 3
#define NTBUS_ID_MOTORPITCH               4
#define NTBUS_ID_MOTORROLL                5
#define NTBUS_ID_MOTORYAW                 6
#define NTBUS_ID_CAMERA                   7
#define NTBUS_ID_LOGGER                   11
#define NTBUS_ID_IMU3                     12

#define NTBUS_STX                         0x80 // 0b 1000 0000
#define NTBUS_SFMASK                      0x70 // 0b 0111 0000
#define NTBUS_IDMASK                      0x0F // 0b 0000 1111

#define NTBUS_FLASH                       0x70 // 0b 0111 0000
#define NTBUS_RESET                       0x50 // 0b 0101 0000
#define NTBUS_SET                         0x40 // 0b 0100 0000
#define NTBUS_GET                         0x30 // 0b 0011 0000
#define NTBUS_TRIGGER                     0x10 // 0b 0001 0000
#define NTBUS_CMD                         0x00 // 0b 0000 0000

// configuration word
// response to NTBUS_CMD_GETCONFIGURATION
#define NTBUS_IMU_CONFIG_MPU6050          0x0001
#define NTBUS_IMU_CONFIG_MPU6000          0x0002
#define NTBUS_IMU_CONFIG_MPU9250          0x0004
#define NTBUS_IMU_CONFIG_MODELUNKNOWN     0x0000
#define NTBUS_IMU_CONFIG_MODELMASK        0x0007

#define NTBUS_IMU_CONFIG_OVERSAMPLED      0x0010
#define NTBUS_IMU_CONFIG_FASTGYRO         0x0020
#define NTBUS_IMU_CONFIG_FASTACC          0x0040
#define NTBUS_IMU_CONFIG_FAST             (NTBUS_IMU_CONFIG_FASTGYRO|NTBUS_IMU_CONFIG_FASTACC)

//IMU RELATED DEFINES
#define NTBUS_IMU_STATUS_IMU_PRESENT      0x80

// GET IMU imu status byte
// response to NTBUS_GET IMU
// also used in NTBUS_CMD_ACCGYRODATA
#define NTBUS_IMU_IMUSTATUS_BASE          0x01  //must always be set, so that status is always >0
#define NTBUS_IMU_IMUSTATE_OK             0x02  //this one is injected, comes from the STorM32 controller, not used by NT imu
#define NTBUS_IMU_IMUSTATUS_GYRODATA_OK   0x04
#define NTBUS_IMU_IMUSTATUS_ACCDATA_OK    0x08
#define NTBUS_IMU_IMUSTATUS_GYRO_CLIPPED  0x10
#define NTBUS_IMU_IMUSTATUS_ACC_CLIPPED   0x20

// NTBUS_GET
// to STorM32: 6xi16 + 1xi16 + 1xu8 + crc = 15+1 bytes
typedef struct __attribute__((__packed__)) {
  int16_t AccX;  //+-4g
  int16_t AccY;  //+-4g
  int16_t AccZ;  //+-4g
  int16_t GyroX; //+-1000°/s
  int16_t GyroY; //+-1000°/s
  int16_t GyroZ; //+-1000°/s
  int16_t Temp;
  uint8_t ImuStatus;
} tNTBusGetImuData;

typedef enum eNTState{
	WAITINGFORSTX,
	WAITINGFORGETCOMMAND,
	WAITINGFORCRC
} eNTState;


typedef struct {
	uint8_t id;
	uint8_t command;
	eNTState state;
} ntBusFrame;

typedef struct {
	uint8_t myID;
	UART_HandleTypeDef* ntUart;
	tNTBusGetImuData* imuData;
	ntBusFrame frame;
}ntImuConf;

typedef struct __attribute__((__packed__)) {
  char Status;
  uint8_t State;
} tNTBusCmdGetStatusData;

typedef enum {
  // general cmds
  NTBUS_CMD_GETSTATUS = 1,
  NTBUS_CMD_GETVERSIONSTR,           //must be supported by any NT module
  NTBUS_CMD_GETBOARDSTR,             //must be supported by any NT module
  NTBUS_CMD_GETCONFIGURATION,        //must be supported by any NT module

  NTBUS_CMD_ACCGYRO1RAWDATA_V1 = 32, //DEPRECTAED
  NTBUS_CMD_ACCGYRO2RAWDATA_V1,	     //DEPRECTAED
  NTBUS_CMD_ACCGYRODATA_V1,	     //DEPRECTAED
  NTBUS_CMD_PIDDATA, //35
  NTBUS_CMD_PARAMETERDATA, //36
  NTBUS_CMD_AHRS1DATA, //37
  NTBUS_CMD_AHRS2DATA,
  NTBUS_CMD_ACCGYRO3RAWDATA_V1, //39 //DEPRECATED
  NTBUS_CMD_ACCGYRO1RAWDATA_V2, //40
  NTBUS_CMD_ACCGYRO2RAWDATA_V2,
  NTBUS_CMD_ACCGYRO3RAWDATA_V2,
  NTBUS_CMD_ACCGYRO1DATA_V2, //43
  NTBUS_CMD_ACCGYRO2DATA_V2,

  NTBUS_CMD_READSETUP = 120,
  NTBUS_CMD_WRITESETUP,
  NTBUS_CMD_STORESETUP,

  NTBUS_CMD_NOTUSED = 0xFF
} NTBUSCMDTYPE;

void ntInit(ntImuConf *conf);
void ntParseCommand(uint8_t command);

#endif
