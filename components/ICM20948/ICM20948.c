#include "ICM20948.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static esp_err_t icm20948_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS);
}

static esp_err_t icm20948_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS);
}

static esp_err_t icm20948_select_bank(i2c_master_dev_handle_t dev_handle, uint8_t bank)
{
    return icm20948_register_write_byte(dev_handle, REG_BANK_SEL, bank << 4);
}

esp_err_t icm20948_init_i2c(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle, i2c_master_dev_handle_t *mag_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ICM20948_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));

    i2c_device_config_t mag_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AK09916_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &mag_config, mag_handle));

    return ESP_OK;
}

uint8_t icm20948_get_who_am_i(i2c_master_dev_handle_t dev_handle)
{
    uint8_t who_am_i = 0;
    icm20948_select_bank(dev_handle, 0);
    icm20948_register_read(dev_handle, WHO_AM_I, &who_am_i, 1);
    return who_am_i;
}

esp_err_t icm20948_init(i2c_master_dev_handle_t dev_handle, i2c_master_dev_handle_t mag_handle)
{
    ESP_LOGI("ICM20948", "Resetting ICM20948...");
    icm20948_select_bank(dev_handle, 0);
    icm20948_register_write_byte(dev_handle, PWR_MGMT_1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(100));

    icm20948_register_write_byte(dev_handle, PWR_MGMT_1, 0x01);
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOGI("ICM20948", "Enabling I2C Bypass for Magnetometer...");
    icm20948_register_write_byte(dev_handle, 0x03, 0x00);
    icm20948_register_write_byte(dev_handle, INT_PIN_CFG, 0x02);
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t mag_id = 0;
    if (icm20948_register_read(mag_handle, 0x01, &mag_id, 1) == ESP_OK)
    {
        ESP_LOGI("ICM20948", "Magnetometer ID (WIA2): 0x%02X (Expected 0x09)", mag_id);
    }
    else
    {
        ESP_LOGE("ICM20948", "Cannot contact Magnetometer! Bypass failed.");
    }

    icm20948_register_write_byte(mag_handle, AK09916_CNTL3, 0x01);
    vTaskDelay(pdMS_TO_TICKS(50));
    icm20948_register_write_byte(mag_handle, AK09916_CNTL2, 0x08); // ±4912 µT

    icm20948_select_bank(dev_handle, 2);
    icm20948_register_write_byte(dev_handle, GYRO_SMPLRT_DIV, 0x04);
    icm20948_register_write_byte(dev_handle, GYRO_CONFIG_1, 0x13); // 500dps
    icm20948_register_write_byte(dev_handle, ACCEL_SMPLRT_DIV_1, 0x00);
    icm20948_register_write_byte(dev_handle, ACCEL_SMPLRT_DIV_2, 0x04);
    icm20948_register_write_byte(dev_handle, ACCEL_CONFIG, 0x11); // 2g

    icm20948_select_bank(dev_handle, 0);
    ESP_LOGI("ICM20948", "ICM20948 & AK09916 Initialization Complete.");
    return ESP_OK;
}

// 初始化I2C和ICM20948，供外部使用
icm20948_handle_t imu_init(void)
{
    // I2C句柄、设备句柄、磁力计句柄
    i2c_master_bus_handle_t bus_handle = NULL;
    i2c_master_dev_handle_t dev_handle = NULL;
    i2c_master_dev_handle_t mag_handle = NULL;

    // 初始化I2C
    icm20948_init_i2c(&bus_handle, &dev_handle, &mag_handle);
    ESP_LOGI("ICM20948", "I2C initialized");

    // 检查ICM20948是否连接
    if (icm20948_get_who_am_i(dev_handle) != ICM20948_WHO_AM_I_VAL)
    {
        ESP_LOGE("ICM20948", "ICM20948 not found!");
        return (icm20948_handle_t){NULL, NULL, NULL};
    }

    // 初始化ICM20948
    icm20948_init(dev_handle, mag_handle);
    ESP_LOGI("ICM20948", "ICM20948 & AK09916 Initialization Complete.");

    icm20948_handle_t handle = {bus_handle, dev_handle, mag_handle};
    return handle;
}

esp_err_t icm20948_read_agm(i2c_master_dev_handle_t dev_handle, i2c_master_dev_handle_t mag_handle, icm20948_data_t *data)
{
    uint8_t status;
    uint8_t raw_data[12];

    icm20948_select_bank(dev_handle, 0);

    ESP_ERROR_CHECK(
        icm20948_register_read(dev_handle, ACCEL_XOUT_H, raw_data, 12));

    data->accel_x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    data->accel_y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    data->accel_z = (int16_t)((raw_data[4] << 8) | raw_data[5]);

    data->gyro_x = (int16_t)((raw_data[6] << 8) | raw_data[7]);
    data->gyro_y = (int16_t)((raw_data[8] << 8) | raw_data[9]);
    data->gyro_z = (int16_t)((raw_data[10] << 8) | raw_data[11]);

    icm20948_register_read(mag_handle, AK09916_ST1, &status, 1);
    if (!(status & 0x01))
        return ESP_FAIL;

    icm20948_register_read(mag_handle, AK09916_HXL, raw_data, 8);

    data->mag_x = (int16_t)((raw_data[1] << 8) | raw_data[0]);
    data->mag_y = (int16_t)((raw_data[3] << 8) | raw_data[2]);
    data->mag_z = (int16_t)((raw_data[5] << 8) | raw_data[4]);

    return ESP_OK;
}

void icm20948_print_rawdata(const icm20948_data_t *data)
{
    ESP_LOGI("ICM20948",
             "ACCEL: X=%d Y=%d Z=%d | GYRO: X=%d Y=%d Z=%d | MAG: X=%d Y=%d Z=%d",
             data->accel_x,
             data->accel_y,
             data->accel_z,
             data->gyro_x,
             data->gyro_y,
             data->gyro_z,
             data->mag_x,
             data->mag_y,
             data->mag_z);

    float ax = data->accel_x / 16384.0f;
    float ay = data->accel_y / 16384.0f;
    float az = data->accel_z / 16384.0f;

    float gx = data->gyro_x / 131.0f;
    float gy = data->gyro_y / 131.0f;
    float gz = data->gyro_z / 131.0f;

    float mx = data->mag_x / 16384.0f;
    float my = data->mag_y / 16384.0f;
    float mz = data->mag_z / 16384.0f;

    ESP_LOGI("ICM20948",
             "Accel[g]: %.3f %.3f %.3f | Gyro[dps]: %.3f %.3f %.3f | MAG: %.3f %.3f %.3f",
             ax, ay, az,
             gx, gy, gz,
             mx, my, mz);
}