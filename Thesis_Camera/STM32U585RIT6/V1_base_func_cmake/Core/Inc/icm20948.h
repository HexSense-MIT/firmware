#ifndef ICM20948_H
#define ICM20948_H

#include "stm32u5xx_hal.h"

/* I2C address (AD0 low = 0x68, high = 0x69); HAL expects 8-bit shifted */
#define ICM20948_ADDR           (0x68 << 1)

#define ICM20948_WHO_AM_I_VAL   0xEA

/* ---- Bank 0 registers ---- */
#define ICM20948_WHO_AM_I       0x00
#define ICM20948_USER_CTRL      0x03
#define ICM20948_PWR_MGMT_1     0x06
#define ICM20948_PWR_MGMT_2     0x07
#define ICM20948_INT_PIN_CFG    0x0F
#define ICM20948_ACCEL_XOUT_H   0x2D
#define ICM20948_TEMP_OUT_H     0x39
#define ICM20948_REG_BANK_SEL   0x7F

/* ---- Bank 2 registers ---- */
#define ICM20948_GYRO_SMPLRT_DIV  0x00
#define ICM20948_GYRO_CONFIG_1    0x01
#define ICM20948_ACCEL_SMPLRT_DIV_2 0x11
#define ICM20948_ACCEL_CONFIG     0x14

/* Sensitivity constants (default FS) */
#define ICM20948_ACCEL_SENS     16384.0f   /* LSB/g  for ±2g  */
#define ICM20948_GYRO_SENS      131.0f     /* LSB/dps for ±250dps */

typedef struct {
    float accel_x, accel_y, accel_z;  /* g     */
    float gyro_x,  gyro_y,  gyro_z;   /* dps   */
    float temp_c;                       /* °C    */
} ICM20948_Data_t;

HAL_StatusTypeDef ICM20948_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef ICM20948_ReadAll(I2C_HandleTypeDef *hi2c, ICM20948_Data_t *data);
void              ICM20948_PrintData(const ICM20948_Data_t *data);

#endif /* ICM20948_H */
