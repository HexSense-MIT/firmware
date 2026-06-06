#include <stdio.h>

#include "icm20948.h"

#define TIMEOUT_MS  100U

void ICM20948_PowrOn(void) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_Delay(10);
}

void ICM20948_PowrOff(void) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
}

static HAL_StatusTypeDef bank_select(I2C_HandleTypeDef *hi2c, uint8_t bank)
{
    uint8_t val = (bank & 0x03) << 4;
    return HAL_I2C_Mem_Write(hi2c, ICM20948_ADDR, ICM20948_REG_BANK_SEL,
                             I2C_MEMADD_SIZE_8BIT, &val, 1, TIMEOUT_MS);
}

static HAL_StatusTypeDef reg_write(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(hi2c, ICM20948_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1, TIMEOUT_MS);
}

static HAL_StatusTypeDef reg_read(I2C_HandleTypeDef *hi2c, uint8_t reg,
                                  uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(hi2c, ICM20948_ADDR, reg,
                            I2C_MEMADD_SIZE_8BIT, buf, len, TIMEOUT_MS);
}

HAL_StatusTypeDef ICM20948_Init(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef ret;
    uint8_t who;

    /* Bank 0: verify device identity */
    ret = bank_select(hi2c, 0);
    if (ret != HAL_OK) return ret;

    ret = reg_read(hi2c, ICM20948_WHO_AM_I, &who, 1);
    if (ret != HAL_OK || who != ICM20948_WHO_AM_I_VAL) return HAL_ERROR;

    /* Reset device, then wake up with auto clock */
    ret = reg_write(hi2c, ICM20948_PWR_MGMT_1, 0x80);  /* device reset */
    if (ret != HAL_OK) return ret;
    HAL_Delay(100);

    ret = reg_write(hi2c, ICM20948_PWR_MGMT_1, 0x01);  /* wake, auto clk */
    if (ret != HAL_OK) return ret;

    ret = reg_write(hi2c, ICM20948_PWR_MGMT_2, 0x00);  /* enable accel+gyro */
    if (ret != HAL_OK) return ret;

    /* Bank 2: configure gyro and accel */
    ret = bank_select(hi2c, 2);
    if (ret != HAL_OK) return ret;

    /* GYRO_SMPLRT_DIV = 10 → ODR = 1125 / (1+10) ≈ 102 Hz */
    ret = reg_write(hi2c, ICM20948_GYRO_SMPLRT_DIV, 0x0A);
    if (ret != HAL_OK) return ret;

    /* GYRO_CONFIG_1: FS=±250dps [bits 2:1]=00, DLPFCFG [bits 5:3]=3 (51Hz), FCHOICE=1 */
    ret = reg_write(hi2c, ICM20948_GYRO_CONFIG_1, (3 << 3) | 0x01);
    if (ret != HAL_OK) return ret;

    /* ACCEL_SMPLRT_DIV_2 = 10 → ODR ≈ 102 Hz */
    ret = reg_write(hi2c, ICM20948_ACCEL_SMPLRT_DIV_2, 0x0A);
    if (ret != HAL_OK) return ret;

    /* ACCEL_CONFIG: FS=±2g [bits 2:1]=00, DLPFCFG [bits 5:3]=3 (50Hz), FCHOICE=1 */
    ret = reg_write(hi2c, ICM20948_ACCEL_CONFIG, (3 << 3) | 0x01);
    if (ret != HAL_OK) return ret;

    /* Back to bank 0 for data reads */
    return bank_select(hi2c, 0);
}

HAL_StatusTypeDef ICM20948_ReadAll(I2C_HandleTypeDef *hi2c, ICM20948_Data_t *data)
{
    /* 14 bytes: ACCEL_XOUT_H … TEMP_OUT_L (0x2D–0x3A) */
    uint8_t raw[14];
    HAL_StatusTypeDef ret = reg_read(hi2c, ICM20948_ACCEL_XOUT_H, raw, 14);
    if (ret != HAL_OK) return ret;

    int16_t ax = (int16_t)((raw[0]  << 8) | raw[1]);
    int16_t ay = (int16_t)((raw[2]  << 8) | raw[3]);
    int16_t az = (int16_t)((raw[4]  << 8) | raw[5]);
    int16_t gx = (int16_t)((raw[6]  << 8) | raw[7]);
    int16_t gy = (int16_t)((raw[8]  << 8) | raw[9]);
    int16_t gz = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t t  = (int16_t)((raw[12] << 8) | raw[13]);

    data->accel_x = ax / ICM20948_ACCEL_SENS;
    data->accel_y = ay / ICM20948_ACCEL_SENS;
    data->accel_z = az / ICM20948_ACCEL_SENS;
    data->gyro_x  = gx / ICM20948_GYRO_SENS;
    data->gyro_y  = gy / ICM20948_GYRO_SENS;
    data->gyro_z  = gz / ICM20948_GYRO_SENS;
    data->temp_c  = (t / 333.87f) + 21.0f;

    return HAL_OK;
}

void ICM20948_PrintData(const ICM20948_Data_t *data)
{
    printf("Accel[g]  X:%6.3f  Y:%6.3f  Z:%6.3f \r\n",
        //    "Gyro[dps] X:%7.2f  Y:%7.2f  Z:%7.2f  |  "
        //    "Temp: %.1f C\r\n",
           (double)data->accel_x, (double)data->accel_y, (double)data->accel_z);
}
