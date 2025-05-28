#pragma once

#include <stdint.h>
#include <stdbool.h>

// imu i2c address 
#define IMU_I2C_ADDRESS      0x68

// imu register addresses (again, values need editing)
#define IMU_REG_PWR_MGMT_1   0x6B
#define IMU_REG_ACCEL_XOUT_H 0x3B
#define IMU_REG_GYRO_XOUT_H  0x43

// little struct for the imu data
typedef struct {
    float accel_x;  // x-axis accel 
    float accel_y;  // y-axis accel 
    float accel_z;  // z-axis accel 
    float gyro_x;   // x-axis spin in degrees
    float gyro_y;   // y-axis spin in deg
    float gyro_z;   // z-axis spin in deg
    float temperature; // celsius
} IMU_Data_t;

// function declarations

/**
 * @brief gets the imu sensor and i2c working
 * @return true if it’s all good, false if not working
 */
bool IMU_Init(void);

/**
 * @brief grabs datafrom imu
 * @param[out] data pointer to where we’ll store data
 * @return true if it worked, false if it didn’t
 */
bool IMU_ReadData(IMU_Data_t* data);

/**
 * @brief turns raw imu numbers into real units like deg/s
 * @param[in,out] data pointer to the data were converting
 */
void IMU_ConvertData(IMU_Data_t* data);

/**
 * @brief tweaks the imu so fully calibrated
 * @return true if calibrations a success, false if it fails
 */
bool IMU_Calibrate(void);

/**
 * @brief checks if the imu’s awake 
 * @return true if it’s awake false if it’s sleeping.
 */
bool IMU_IsConnected(void);

/**
 * @brief pulls the temp from the imu
 * @return temp in degrees celsius
 */
float IMU_ReadTemperature(void);

/**
 * @brief grabs the accel data from the imu
 * @param[out] accel_x where x-axis accel goes
 * @param[out] accel_y where y-axis accel goes
 * @param[out] accel_z where z-axis accel goes
 * @return true if we got it, false if not
 */
bool IMU_ReadAccelerometer(float* accel_x, float* accel_y, float* accel_z);

/**
 * @brief grabs the gyro data from the imu
 * @param[out] gyro_x where x-axis spin goes
 * @param[out] gyro_y where y-axis spin goes
 * @param[out] gyro_z where z-axis spin goes
 * @return true if it’s good, false if it busted
 */
bool IMU_ReadGyroscope(float* gyro_x, float* gyro_y, float* gyro_z);

/**
 * @brief pulls raw data off imu
 * @param[out] raw_data where we put the raw bytes
 * @return true if it worked, false if it didn’t
 *
bool IMU_ReadRawData(uint8_t* raw_data);

/**
 * @brief sends some data to the imu
 * @param[in] reg register we’re using
 * @param[in] data what were sending
 * @return true if it went through, false if it failed
 */
bool IMU_WriteData(uint8_t reg, uint8_t data);

/**
 * @brief reads one byte from the imu
 * @param[in] reg register used
 * @param[out] data where we put the byte
 * @return true if it’s went thru, false if it’s not
 */
bool IMU_ReadByte(uint8_t reg, uint8_t* data);

/**
 * @brief grabs a bunch of bytes from the imu.
 * @param[in] reg register to start at.
 * @param[out] data where the bytes go.
 * @param[in] length how many bytes we’re after.
 * @return true if it’s a win, false if it’s a loss.
 */
bool IMU_ReadBytes(uint8_t reg, uint8_t* data, uint16_t length);

/**
 * @brief writes one byte to the imu
 * @param[in] reg register were writing to
 * @param[in] data the byte were sending
 * @return true if it went, false if it didn’t
 */
bool IMU_WriteByte(uint8_t reg, uint8_t data);

/**
 * @brief sends a bunch of bytes to the imu
 * @param[in] reg register to start at
 * @param[in] data pointer to the bytes were sending
 * @param[in] length how many bytes to write
 * @return true if it worked, false if it fails
 */
bool IMU_WriteBytes(uint8_t reg, const uint8_t* data, uint16_t length);

/**
 * @brief delay
 * @param[in] ms how many milliseconds to wait
 */
void IMU_Delay(uint32_t ms);

/**
 * @brief hits the reset button on the imu
 * @return true if it’s back to square one, false if it didn't quit
 */
bool IMU_Reset(void);
