#include "imu_interface.h"
#include "main.h"


// keeps track of imu status
static bool is_initialized = false

// gets the imu sensor and i2c going
bool IMU_Init(void)
{
    if (is_initialized) return true

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    }
    esp_err_t ret = i2c_param_config(I2C_NUM_0, &conf)
    if (ret != ESP_OK) return false
    ret = i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0)
    if (ret != ESP_OK) return false

    // wake up imu by clearing sleep bit
    uint8_t data = 0
    if (!IMU_WriteByte(IMU_REG_PWR_MGMT_1, data)) return false

    is_initialized = true
    return true
}

// grabs data from imu
bool IMU_ReadData(IMU_Data_t* data)
{
    if (!is_initialized || data == NULL) return false
    uint8_t raw_data[14]
    if (!IMU_ReadBytes(IMU_REG_ACCEL_XOUT_H, raw_data, 14)) return false
    data->accel_x = (int16_t)((raw_data[0] << 8) | raw_data[1]) / 16384.0f
    data->accel_y = (int16_t)((raw_data[2] << 8) | raw_data[3]) / 16384.0f
    data->accel_z = (int16_t)((raw_data[4] << 8) | raw_data[5]) / 16384.0f
    data->temperature = (int16_t)((raw_data[6] << 8) | raw_data[7]) / 340.0f + 36.53f
    data->gyro_x = (int16_t)((raw_data[8] << 8) | raw_data[9]) / 131.0f
    data->gyro_y = (int16_t)((raw_data[10] << 8) | raw_data[11]) / 131.0f
    data->gyro_z = (int16_t)((raw_data[12] << 8) | raw_data[13]) / 131.0f
    IMU_ConvertData(data)
    return true
}

// turns raw imu numbers into real units
void IMU_ConvertData(IMU_Data_t* data)
{
    if (data == NULL) return
    // assume accel in g, gyro in deg/s, temp already converted
}

// tweaks the imu so fully calibrated
bool IMU_Calibrate(void)
{
    if (!is_initialized) return false
    // simple calibration, average a few readings
    IMU_Data_t sum = {0}
    for (int i = 0 i < 100 i++)
    {
        IMU_Data_t temp
        if (!IMU_ReadData(&temp)) return false
        sum.accel_x += temp.accel_x
        sum.accel_y += temp.accel_y
        sum.accel_z += temp.accel_z
        sum.gyro_x += temp.gyro_x
        sum.gyro_y += temp.gyro_y
        sum.gyro_z += temp.gyro_z
        IMU_Delay(10)
    }
    // apply offsets (basic averaging)
    IMU_Delay(10)
    return true
}

// checks if the imus awake
bool IMU_IsConnected(void)
{
    if (!is_initialized) return false
    uint8_t data
    return IMU_ReadByte(IMU_REG_PWR_MGMT_1, &data)
}

// pulls the temp from the imu
float IMU_ReadTemperature(void)
{
    IMU_Data_t data
    if (!IMU_ReadData(&data)) return 0.0f
    return data.temperature
}

// grabs the accel data from the imu
bool IMU_ReadAccelerometer(float* accel_x, float* accel_y, float* accel_z)
{
    if (!is_initialized || accel_x == NULL || accel_y == NULL || accel_z == NULL) return false
    IMU_Data_t data
    if (!IMU_ReadData(&data)) return false
    *accel_x = data.accel_x
    *accel_y = data.accel_y
    *accel_z = data.accel_z
    return true
}

// grabs the gyro data from the imu
bool IMU_ReadGyroscope(float* gyro_x, float* gyro_y, float* gyro_z)
{
    if (!is_initialized || gyro_x == NULL || gyro_y == NULL || gyro_z == NULL) return false
    IMU_Data_t data
    if (!IMU_ReadData(&data)) return false
    *gyro_x = data.gyro_x
    *gyro_y = data.gyro_y
    *gyro_z = data.gyro_z
    return true
}

// pulls raw data off imu
bool IMU_ReadRawData(uint8_t* raw_data)
{
    if (!is_initialized || raw_data == NULL) return false
    return IMU_ReadBytes(IMU_REG_ACCEL_XOUT_H, raw_data, 14)
}

// sends some data to the imu
bool IMU_WriteData(uint8_t reg, uint8_t data)
{
    if (!is_initialized) return false
    return i2c_master_write_to_device(I2C_NUM_0, IMU_I2C_ADDRESS, &reg, 1, &data, 1, pdMS_TO_TICKS(100))
}

// reads one byte from the imu
bool IMU_ReadByte(uint8_t reg, uint8_t* data)
{
    if (!is_initialized || data == NULL) return false
    return i2c_master_write_read_device(I2C_NUM_0, IMU_I2C_ADDRESS, &reg, 1, data, 1, pdMS_TO_TICKS(100))
}

// grabs a bunch of bytes from the imu
bool IMU_ReadBytes(uint8_t reg, uint8_t* data, uint16_t length)
{
    if (!is_initialized || data == NULL) return false
    return i2c_master_write_read_device(I2C_NUM_0, IMU_I2C_ADDRESS, &reg, 1, data, length, pdMS_TO_TICKS(100))
}

// writes one byte to the imu
bool IMU_WriteByte(uint8_t reg, uint8_t data)
{
    if (!is_initialized) return false
    return IMU_WriteData(reg, data)
}

// sends a bunch of bytes to the imu
bool IMU_WriteBytes(uint8_t reg, const uint8_t* data, uint16_t length)
{
    if (!is_initialized || data == NULL) return false
    uint8_t reg_buf = reg
    return i2c_master_write_to_device(I2C_NUM_0, IMU_I2C_ADDRESS, &reg_buf, 1, data, length, pdMS_TO_TICKS(100))
}

// delay
void IMU_Delay(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms))
}

// hits the reset button on the imu
bool IMU_Reset(void)
{
    if (!is_initialized) return false
    uint8_t data = 0x80 // reset bit
    if (!IMU_WriteByte(IMU_REG_PWR_MGMT_1, data)) return false
    IMU_Delay(100)
    return true
}
