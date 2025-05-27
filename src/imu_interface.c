#include "../include/imu_interface.h"


// Function Implementations

/**
 * @brief Initialize the IMU sensor and I2C interface.
 * @return true if initialization was successful, false otherwise.
 */
bool IMU_Init(void) {
    // Initialize I2C interface
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    if (i2c_param_config(I2C_NUM_0, &conf) != ESP_OK) {
        return false;
    }
    if (i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0) != ESP_OK) {
        return false;
    }

    // Initialize IMU sensor (e.g., configure registers)
    // Add specific IMU initialization code here

    return true;
}

/**
 * @brief Read raw data from the IMU sensor.
 * @param[out] data Pointer to IMU_Data_t structure to store the data.
 * @return true if read was successful, false otherwise.
 */
bool IMU_ReadData(IMU_Data_t* data) {
    // Read raw accelerometer data
    if (!IMU_ReadAccelerometer(&data->accel_x, &data->accel_y, &data->accel_z)) {
        return false;
    }

    // Read raw gyroscope data
    if (!IMU_ReadGyroscope(&data->gyro_x, &data->gyro_y, &data->gyro_z)) {
        return false;
    }

    // Read temperature data
    data->temperature = IMU_ReadTemperature();

    // Read raw data bytes (if applicable)
    uint8_t raw_data[6]; // Adjust size based on IMU specifications
    if (!IMU_ReadRawData(raw_data)) {
        return false;
    }
    
    // Store raw data in the structure (if needed)
    data->raw_data = raw_data;

    return true;
}

/**
 * @brief Convert raw IMU data to physical units (g, deg/s, etc).
 * @param[in,out] data Pointer to IMU_Data_t structure to convert.
 */
void IMU_ConvertData(IMU_Data_t* data) {
    // Convert accelerometer data from raw to g (assuming 16g full scale)
    data->accel_x = data->accel_x / 32768.0f * 16.0f;
    data->accel_y = data->accel_y / 32768.0f * 16.0f;
    data->accel_z = data->accel_z / 32768.0f * 16.0f;

    // Convert gyroscope data from raw to deg/s (assuming 2000 dps full scale)
    data->gyro_x = data->gyro_x / 32768.0f * 2000.0f;
    data->gyro_y = data->gyro_y / 32768.0f * 2000.0f;
    data->gyro_z = data->gyro_z / 32768.0f * 2000.0f;

    // Temperature is already in degrees Celsius
}

/**
 * @brief Calibrate the IMU sensor.
 * @return true if calibration was successful, false otherwise.
 */
bool IMU_Calibrate(void) {
    // Implement calibration logic here
    // This could involve reading multiple samples and averaging them
    // or applying specific calibration algorithms based on the IMU type

    // Example: Simple zeroing of accelerometer and gyroscope
    IMU_Data_t data;
    if (!IMU_ReadData(&data)) {
        return false;
    }

    // Store calibration offsets (this is just an example, actual calibration may vary)
    imu_calibration_offset.accel_x = -data.accel_x;
    imu_calibration_offset.accel_y = -data.accel_y;
    imu_calibration_offset.accel_z = -data.accel_z;
    imu_calibration_offset.gyro_x = -data.gyro_x;
    imu_calibration_offset.gyro_y = -data.gyro_y;
    imu_calibration_offset.gyro_z = -data.gyro_z;

    return true;
}

/**
 * @brief Check if the IMU sensor is connected and responding.
 * @return true if the sensor is connected, false otherwise.
 */
bool IMU_IsConnected(void) {
    uint8_t data;
    // Attempt to read a known register (e.g., WHO_AM_I register)
    if (IMU_ReadByte(IMU_WHO_AM_I_REG, &data)) {
        // Check if the read value matches the expected value for the IMU
        return (data == IMU_EXPECTED_WHO_AM_I);
    }
    return false;
}

/**
 * @brief Read the temperature from the IMU sensor.
 * @return Temperature in degrees Celsius.
 */
float IMU_ReadTemperature(void) {
    uint8_t temp_data[2];
    if (IMU_ReadBytes(IMU_TEMP_REG, temp_data, 2)) {
        // Combine the two bytes into a single temperature value
        int16_t temp_raw = (temp_data[0] << 8) | temp_data[1];
        // Convert to degrees Celsius (assuming a specific conversion factor)
        return (float)temp_raw / 340.0f + 36.53f; // Example conversion
    }
    return 0.0f; // Return 0 if read failed
}

/**
 * @brief Read the accelerometer data from the IMU sensor.
 * @param[out] accel_x Pointer to store the X-axis acceleration.
 * @param[out] accel_y Pointer to store the Y-axis acceleration.
 * @param[out] accel_z Pointer to store the Z-axis acceleration.
 * @return true if read was successful, false otherwise.
 */
bool IMU_ReadAccelerometer(float* accel_x, float* accel_y, float* accel_z) {
    uint8_t accel_data[6];
    if (IMU_ReadBytes(IMU_ACCEL_REG, accel_data, 6)) {
        // Combine the two bytes for each axis into a single value
        int16_t ax = (accel_data[0] << 8) | accel_data[1];
        int16_t ay = (accel_data[2] << 8) | accel_data[3];
        int16_t az = (accel_data[4] << 8) | accel_data[5];

        // Convert to g (assuming a specific conversion factor)
        *accel_x = (float)ax / 32768.0f * 16.0f; // Example conversion
        *accel_y = (float)ay / 32768.0f * 16.0f;
        *accel_z = (float)az / 32768.0f * 16.0f;

        return true;
    }
    return false;
}

/**
 * @brief Read the gyroscope data from the IMU sensor.
 * @param[out] gyro_x Pointer to store the X-axis angular velocity.
 * @param[out] gyro_y Pointer to store the Y-axis angular velocity.
 * @param[out] gyro_z Pointer to store the Z-axis angular velocity.
 * @return true if read was successful, false otherwise.
 */
bool IMU_ReadGyroscope(float* gyro_x, float* gyro_y, float* gyro_z) {
    uint8_t gyro_data[6];
    if (IMU_ReadBytes(IMU_GYRO_REG, gyro_data, 6)) {
        // Combine the two bytes for each axis into a single value
        int16_t gx = (gyro_data[0] << 8) | gyro_data[1];
        int16_t gy = (gyro_data[2] << 8) | gyro_data[3];
        int16_t gz = (gyro_data[4] << 8) | gyro_data[5];

        // Convert to degrees per second (assuming a specific conversion factor)
        *gyro_x = (float)gx / 32768.0f * 2000.0f; // Example conversion
        *gyro_y = (float)gy / 32768.0f * 2000.0f;
        *gyro_z = (float)gz / 32768.0f * 2000.0f;

        return true;
    }
    return false;
}

/**
 * @brief Read the raw data from the IMU sensor.
 * @param[out] raw_data Pointer to store the raw data.
 * @return true if read was successful, false otherwise.
 */
bool IMU_ReadRawData(uint8_t* raw_data) {
    // Read raw data bytes from a specific register (e.g., IMU_RAW_DATA_REG)
    // Adjust the register address and length based on the IMU specifications
    return IMU_ReadBytes(IMU_RAW_DATA_REG, raw_data, IMU_RAW_DATA_LENGTH);
}

/**
 * @brief Write data to the IMU sensor.
 * @param[in] reg Register address to write to.
 * @param[in] data Data to write.
 * @return true if write was successful, false otherwise.
 */
bool IMU_WriteData(uint8_t reg, uint8_t data) {
    // Write a single byte to the specified register
    return IMU_WriteByte(reg, data);
}

/**
 * @brief Read a single byte from the IMU sensor.
 * @param[in] reg Register address to read from.
 * @param[out] data Pointer to store the read data.
 * @return true if read was successful, false otherwise.
 */
bool IMU_ReadByte(uint8_t reg, uint8_t* data) {
    // Read a single byte from the specified register
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (IMU_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (IMU_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_ACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    return (ret == ESP_OK);
}

/**
 * @brief Read multiple bytes from the IMU sensor.
 * @param[in] reg Register address to read from.
 * @param[out] data Pointer to store the read data.
 * @param[in] length Number of bytes to read.
 * @return true if read was successful, false otherwise.
 */
bool IMU_ReadBytes(uint8_t reg, uint8_t* data, uint16_t length) {
    // Read multiple bytes from the specified register
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (IMU_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (IMU_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
    
    for (uint16_t i = 0; i < length; i++) {
        if (i == length - 1) {
            i2c_master_read_byte(cmd, &data[i], I2C_MASTER_NACK); // Last byte
        } else {
            i2c_master_read_byte(cmd, &data[i], I2C_MASTER_ACK); // Not last byte
        }
    }
    
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    return (ret == ESP_OK);
}

/**
 * @brief Write a single byte to the IMU sensor.
 * @param[in] reg Register address to write to.
 * @param[in] data Data to write.
 * @return true if write was successful, false otherwise.
 */
bool IMU_WriteByte(uint8_t reg, uint8_t data) {
    // Write a single byte to the specified register
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (IMU_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    return (ret == ESP_OK);
}

/**
 * @brief Write multiple bytes to the IMU sensor.
 * @param[in] reg Register address to write to.
 * @param[in] data Pointer to data to write.
 * @param[in] length Number of bytes to write.
 * @return true if write was successful, false otherwise.
 */
bool IMU_WriteBytes(uint8_t reg, const uint8_t* data, uint16_t length) {
    // Write multiple bytes to the specified register
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (IMU_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    
    for (uint16_t i = 0; i < length; i++) {
        i2c_master_write_byte(cmd, data[i], true);
    }
    
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    return (ret == ESP_OK);
}

/**
 * @brief Delay function for the IMU sensor.
 * @param[in] ms Delay in milliseconds.
 */
void IMU_Delay(uint32_t ms) {
    // Use the FreeRTOS delay function
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/**
 * @brief Reset the IMU sensor.
 * @return true if reset was successful, false otherwise.
 */
bool IMU_Reset(void) {
    // Write to the reset register (if applicable)
    // This is an example, actual reset logic may vary based on the IMU
    return IMU_WriteByte(IMU_RESET_REG, IMU_RESET_VALUE);
}