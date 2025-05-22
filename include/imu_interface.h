#pragma once

#include <stdint.h>
#include <stdbool.h>


// Sensor Interface Module (IMU Data Acquisition)
//
// This module will be responsible for interfacing with the IMU sensor
// and acquiring the necessary data for the robot's balancing algorithm.
// It will handle the I2C communication and data processing.
// The module will include functions for initializing the sensor, reading
// data, and converting raw data into usable formats.
//
//  HAL/Scheduler



// IMU I2C address (example for MPU6050, change as needed)
#define IMU_I2C_ADDRESS      0x68

// IMU Register Addresses (example for MPU6050)
#define IMU_REG_PWR_MGMT_1   0x6B
#define IMU_REG_ACCEL_XOUT_H 0x3B
#define IMU_REG_GYRO_XOUT_H  0x43

// IMU Data Structure
typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temperature;
} IMU_Data_t;

// Function Declarations

/**
 * @brief Initialize the IMU sensor and I2C interface.
 * @return true if initialization was successful, false otherwise.
 */
bool IMU_Init(void);

/**
 * @brief Read raw data from the IMU sensor.
 * @param[out] data Pointer to IMU_Data_t structure to store the data.
 * @return true if read was successful, false otherwise.
 */
bool IMU_ReadData(IMU_Data_t* data);

/**
 * @brief Convert raw IMU data to physical units (g, deg/s, etc).
 * @param[in,out] data Pointer to IMU_Data_t structure to convert.
 */
void IMU_ConvertData(IMU_Data_t* data);

/**
 * @brief Calibrate the IMU sensor.
 * @return true if calibration was successful, false otherwise.
 */
bool IMU_Calibrate(void);

/**
 * @brief Check if the IMU sensor is connected and responding.
 * @return true if the sensor is connected, false otherwise.
 */
bool IMU_IsConnected(void);

/**
 * @brief Read the temperature from the IMU sensor.
 * @return Temperature in degrees Celsius.
 */
float IMU_ReadTemperature(void);

/**
 * @brief Read the accelerometer data from the IMU sensor.
 * @param[out] accel_x Pointer to store the X-axis acceleration.
 * @param[out] accel_y Pointer to store the Y-axis acceleration.
 * @param[out] accel_z Pointer to store the Z-axis acceleration.
 * @return true if read was successful, false otherwise.
 */
bool IMU_ReadAccelerometer(float* accel_x, float* accel_y, float* accel_z);

/**
 * @brief Read the gyroscope data from the IMU sensor.
 * @param[out] gyro_x Pointer to store the X-axis angular velocity.
 * @param[out] gyro_y Pointer to store the Y-axis angular velocity.
 * @param[out] gyro_z Pointer to store the Z-axis angular velocity.
 * @return true if read was successful, false otherwise.
 */

 bool IMU_ReadGyroscope(float* gyro_x, float* gyro_y, float* gyro_z);
/**
 * @brief Read the raw data from the IMU sensor.
 * @param[out] raw_data Pointer to store the raw data.
 * @return true if read was successful, false otherwise.
 */

 bool IMU_ReadRawData(uint8_t* raw_data);
/**
 * @brief Write data to the IMU sensor.
 * @param[in] reg Register address to write to.
 * @param[in] data Data to write.
 * @return true if write was successful, false otherwise.
 */

 bool IMU_WriteData(uint8_t reg, uint8_t data);
/**
 * @brief Read a single byte from the IMU sensor.
 * @param[in] reg Register address to read from.
 * @param[out] data Pointer to store the read data.
 * @return true if read was successful, false otherwise.
 */

 bool IMU_ReadByte(uint8_t reg, uint8_t* data);
/**
 * @brief Read multiple bytes from the IMU sensor.
 * @param[in] reg Register address to read from.
 * @param[out] data Pointer to store the read data.
 * @param[in] length Number of bytes to read.
 * @return true if read was successful, false otherwise.
 */

 bool IMU_ReadBytes(uint8_t reg, uint8_t* data, uint16_t length);
/**
 * @brief Write a single byte to the IMU sensor.
 * @param[in] reg Register address to write to.
 * @param[in] data Data to write.
 * @return true if write was successful, false otherwise.
 */
 bool IMU_WriteByte(uint8_t reg, uint8_t data);

 /**
 * @brief Write multiple bytes to the IMU sensor.
 * @param[in] reg Register address to write to.
 * @param[in] data Pointer to data to write.
 * @param[in] length Number of bytes to write.
 * @return true if write was successful, false otherwise.
 */
 bool IMU_WriteBytes(uint8_t reg, const uint8_t* data, uint16_t length);

 /**
 * @brief Delay function for the IMU sensor.
 * @param[in] ms Delay in milliseconds.
 */
void IMU_Delay(uint32_t ms);

/**
 * @brief Reset the IMU sensor.
 * @return true if reset was successful, false otherwise.
 */
bool IMU_Reset(void);