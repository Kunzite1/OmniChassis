#ifndef ICM20948_H
#define ICM20948_H

#include "driver/i2c_master.h"
#include "esp_err.h"

// 引脚配置
#define I2C_MASTER_SCL_IO 1
#define I2C_MASTER_SDA_IO 2

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000

#define ICM20948_SENSOR_ADDR 0x68
#define AK09916_ADDR 0x0C

#define ICM20948_WHO_AM_I_VAL 0xEA

#define I2C_MASTER_TIMEOUT_MS 1000

#define REG_BANK_SEL 0x7F
#define WHO_AM_I 0x00
#define PWR_MGMT_1 0x06
#define PWR_MGMT_2 0x07
#define ACCEL_XOUT_H 0x2D
#define GYRO_XOUT_H 0x33

#define GYRO_SMPLRT_DIV 0x00
#define GYRO_CONFIG_1 0x01
#define ACCEL_SMPLRT_DIV_1 0x10
#define ACCEL_SMPLRT_DIV_2 0x11
#define ACCEL_CONFIG 0x14

#define MAGNETOMETER_XOUT_H 0x03

#define AK09916_WIA2 0x01
#define AK09916_ST1 0x10
#define AK09916_HXL 0x11
#define AK09916_CNTL2 0x31
#define AK09916_CNTL3 0x32

#define INT_PIN_CFG 0x0F

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
} icm20948_data_t;

typedef struct
{
    i2c_master_bus_handle_t bus_handle; // I2C总句柄
    i2c_master_dev_handle_t dev_handle; // ICM20948句柄
    i2c_master_dev_handle_t mag_handle; // AK09916句柄
} icm20948_handle_t;

icm20948_handle_t imu_init(void);

esp_err_t icm20948_init(i2c_master_dev_handle_t dev_handle, i2c_master_dev_handle_t mag_handle);
esp_err_t icm20948_read_agm(i2c_master_dev_handle_t dev_handle, i2c_master_dev_handle_t mag_handle, icm20948_data_t *data);
uint8_t icm20948_get_who_am_i(i2c_master_dev_handle_t dev_handle);
esp_err_t icm20948_init_i2c(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle, i2c_master_dev_handle_t *mag_handle);
void icm20948_print_rawdata(const icm20948_data_t *data);

#ifdef __cplusplus
}
#endif

#endif