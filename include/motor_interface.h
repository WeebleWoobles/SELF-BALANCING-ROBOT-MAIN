#pragma once
//#include <cstdint>
#include <stdbool.h>
#include <stdint.h>
#include "control_logic.h"


// Motor Driver Interface (Actuator Control)
//
// This module will be responsible for controlling the motors of the robot.
// It will interface with the motor driver hardware and implement
// functions for setting motor speeds and directions. The module will
// receive commands from the main control logic and translate them into
// appropriate signals for the motor driver. It will also include
// functions for monitoring motor status and handling any errors or
// faults that may occur during operation.
//
//  HAL/Scheduler


// Motor Interface Parameters
#define MOTOR_MAX_SPEED        255     // Max PWM value for motor speed
#define MOTOR_MIN_SPEED        0       // Min PWM value for motor speed
#define MOTOR_NUM_MOTORS       2       // Number of motors (left, right)

// Motor Status Structure
typedef struct {
    float speed_left;      // Current speed of left motor
    float speed_right;     // Current speed of right motor
    bool fault_left;       // Fault status for left motor
    bool fault_right;      // Fault status for right motor
    bool enabled;          // Motors enabled/disabled
} MotorStatus_t;

// Function Declarations

/**
 * @brief Initialize the motor interface module.
 */
void MotorInterface_Init(void);

/**
 * @brief Set the motor speeds based on control output.
 * @param[in] output Pointer to ControlOutput_t structure.
 */
void MotorInterface_SetSpeeds(const ControlOutput_t* output);

/**
 * @brief Enable or disable the motors.
 * @param[in] enable True to enable, false to disable.
 */
void MotorInterface_Enable(bool enable);

/**
 * @brief Check if the motors are enabled.
 * @return True if enabled, false otherwise.
 */
bool MotorInterface_IsEnabled(void);

/**
 * @brief Get the current motor status.
 * @param[out] status Pointer to MotorStatus_t structure to fill.
 */
void MotorInterface_GetStatus(MotorStatus_t* status);

/**
 * @brief Reset the motor interface (e.g., after a fault or reset event).
 */
void MotorInterface_Reset(void);

/**
 * @brief Check for motor faults.
 * @return True if a fault is detected, false otherwise.
 */
bool MotorInterface_HasFault(void);

/**
 * @brief Set the motor speed limit.
 * @param[in] limit Speed limit (0 to MOTOR_MAX_SPEED).
 */
void MotorInterface_SetSpeedLimit(uint8_t limit);

/**
 * @brief Get the current motor speed limit.
 * @return Current speed limit.
 */
uint8_t MotorInterface_GetSpeedLimit(void);

/**
 * @brief Set the motor direction.
 * @param[in] left_direction Direction for left motor (true for forward, false for reverse).
 * @param[in] right_direction Direction for right motor (true for forward, false for reverse).
 */
void MotorInterface_SetDirection(bool left_direction, bool right_direction);

/**
 * @brief Get the current motor direction.
 * @param[out] left_direction Pointer to store left motor direction.
 * @param[out] right_direction Pointer to store right motor direction.
 */
void MotorInterface_GetDirection(bool* left_direction, bool* right_direction);

/**
 * @brief Set the motor PWM frequency.
 * @param[in] frequency PWM frequency in Hz.
 */
void MotorInterface_SetPWMFrequency(uint32_t frequency);

/**
 * @brief Get the current motor PWM frequency.
 * @return Current PWM frequency in Hz.
 */
uint32_t MotorInterface_GetPWMFrequency(void);

/**
 * @brief Set the motor acceleration and deceleration rates.
 * @param[in] acceleration Acceleration rate in units per second.
 * @param[in] deceleration Deceleration rate in units per second.
 */
void MotorInterface_SetAcceleration(float acceleration, float deceleration);

/**
 * @brief Get the current motor acceleration and deceleration rates.
 * @param[out] acceleration Pointer to store acceleration rate.
 * @param[out] deceleration Pointer to store deceleration rate.
 */
void MotorInterface_GetAcceleration(float* acceleration, float* deceleration);

/**
 * @brief Set the motor control mode (e.g., speed, position).
 * @param[in] mode Control mode (e.g., speed, position).
 */
void MotorInterface_SetControlMode(uint8_t mode);

/**
 * @brief Get the current motor control mode.
 * @return Current control mode.
 */
uint8_t MotorInterface_GetControlMode(void);

/**
 * @brief Set the motor fault handling mode (e.g., stop, continue).
 * @param[in] mode Fault handling mode (e.g., stop, continue).
 */
void MotorInterface_SetFaultHandlingMode(uint8_t mode);

/**
 * @brief Get the current motor fault handling mode.
 * @return Current fault handling mode.
 */
uint8_t MotorInterface_GetFaultHandlingMode(void);

/**
 * @brief Set the motor fault reset command.
 * @param[in] reset True to reset faults, false to ignore.
 */
void MotorInterface_SetFaultReset(bool reset);

