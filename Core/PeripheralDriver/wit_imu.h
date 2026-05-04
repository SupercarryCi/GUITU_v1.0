#ifndef WIT_IMU_H
#define WIT_IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* JY901P UART主动输出帧固定11字节：
 * byte0  : 0x55 帧头
 * byte1  : TYPE 数据类型
 * byte2~9: 8字节数据，协议里所有16位数据都是低字节在前
 * byte10 : SUM，前10字节累加后取低8位
 */
#define WIT_IMU_FRAME_LEN        11U
#define WIT_IMU_FRAME_HEAD       0x55U

#define WIT_IMU_TYPE_TIME        0x50U
#define WIT_IMU_TYPE_ACC         0x51U
#define WIT_IMU_TYPE_GYRO        0x52U
#define WIT_IMU_TYPE_ANGLE       0x53U
#define WIT_IMU_TYPE_QUAT        0x59U
#define WIT_IMU_TYPE_READ        0x5FU

/* 数据有效标志。 */
#define WIT_IMU_DATA_TIME        (1U << 0)
#define WIT_IMU_DATA_ACC         (1U << 1)
#define WIT_IMU_DATA_GYRO        (1U << 2)
#define WIT_IMU_DATA_ANGLE       (1U << 3)
#define WIT_IMU_DATA_QUAT        (1U << 4)
#define WIT_IMU_DATA_READ        (1U << 5)

#define WIT_IMU_GRAVITY_MPS2     9.80665f

#define WIT_IMU_REG_SAVE         0x00U
#define WIT_IMU_REG_RSW          0x02U
#define WIT_IMU_REG_RRATE        0x03U
#define WIT_IMU_REG_BANDWIDTH    0x1FU
#define WIT_IMU_REG_READADDR     0x27U
#define WIT_IMU_REG_KEY          0x69U

#define WIT_IMU_KEY_UNLOCK       0xB588U
#define WIT_IMU_SAVE_NOW         0x0000U
#define WIT_IMU_RSW_ACC_GYRO     0x0006U
#define WIT_IMU_RSW_MOTION_QUAT  0x020EU  /* ACC + GYRO + ANGLE + QUAT */
#define WIT_IMU_RSW_FULL         0x020FU
#define WIT_IMU_RRATE_200HZ      0x000BU
#define WIT_IMU_BANDWIDTH_98HZ   0x0002U

typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} WitImuVector3;

typedef struct
{
  int16_t q0;
  int16_t q1;
  int16_t q2;
  int16_t q3;
} WitImuQuat;

typedef struct
{
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint16_t millisecond;
} WitImuTime;

typedef struct
{
  WitImuTime time;        /* 原始时间字段：YY MM DD HH MN SS MS   */
  WitImuVector3 acc;      /* 原始加速度：raw / 32768 * 16g        */
  WitImuVector3 gyro;     /* 原始角速度：raw / 32768 * 2000 deg/s */
  WitImuVector3 angle;    /* 原始角度：raw / 32768 * 180 deg      */
  WitImuQuat quat;        /* 原始四元数：raw / 32768              */
  int16_t temp;           /* 原始温度：raw / 100 degC             */
  uint16_t read_reg[4];    /* 读取寄存器返回的连续4个寄存器值       */
  uint8_t read_addr;       /* read_reg[0]对应的寄存器地址           */
  uint8_t valid;          /* 当前已经收到过哪些类型的数据           */
  uint32_t frame_count;
  uint32_t time_count;
  uint32_t acc_count;
  uint32_t gyro_count;
  uint32_t angle_count;
  uint32_t quat_count;
  uint32_t read_count;
  uint32_t unknown_count;
  uint32_t checksum_error;
  uint8_t last_type;
} WitImuData;

void WitImu_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef WitImu_StartReceiveIT(void);
uint8_t WitImu_UartRxCpltCallback(UART_HandleTypeDef *huart);
void WitImu_UartErrorCallback(UART_HandleTypeDef *huart);
uint8_t WitImu_TakeSampleReady(void);
uint8_t WitImu_CopyData(WitImuData *data);
uint8_t WitImu_ParseByte(uint8_t byte);
uint16_t WitImu_ParseBuffer(const uint8_t *data, uint16_t len);
const WitImuData *WitImu_GetData(void);
float WitImu_AccRawToMps2(int16_t raw);
float WitImu_GyroRawToRps(int16_t raw);
float WitImu_AngleRawToDeg(int16_t raw);
float WitImu_QuatRawToFloat(int16_t raw);
HAL_StatusTypeDef WitImu_WriteRegister(uint8_t reg, uint16_t value);
HAL_StatusTypeDef WitImu_Unlock(void);
HAL_StatusTypeDef WitImu_SaveConfig(void);
HAL_StatusTypeDef WitImu_SetOutput(uint16_t rsw);
HAL_StatusTypeDef WitImu_SetReportRate(uint16_t rrate);
HAL_StatusTypeDef WitImu_SetGyroBandwidth(uint16_t bandwidth);
HAL_StatusTypeDef WitImu_ConfigureJY901P(void);
HAL_StatusTypeDef WitImu_SetGyroBandwidth98Hz(void);
HAL_StatusTypeDef WitImu_ReadRegisters(uint8_t start_addr);

#ifdef __cplusplus
}
#endif

#endif /* WIT_IMU_H */
