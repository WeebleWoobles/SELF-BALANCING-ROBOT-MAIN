#pragma once

#include <stdint.h>
#include <stdbool.h>

// motor driver interface (actuator control)
//
// this lil module takes care of the robot motors
// it talks to the motor driver hardware and sets speeds and directions
// gets commands from control logic and turns them into motor signals
// also keeps an eye on motor status and deals with any oopsies

// motor interface params
#define MOTOR_MAX_SPEED        255     // max pwm value for motor speed
#define MOTOR_MIN_SPEED        0       // min pwm value for motor speed
#define MOTOR_NUM_MOTORS       2       // we got left and right motors

// motor status structure
typedef struct {
    float speed_left      // current speed of left motor
    float speed_right     // current speed of right motor
    bool fault_left       // fault status for left motor
    bool fault_right      // fault status for right motor
    bool enabled          // motors on or off
} MotorStatus_t;

// control output structure
typedef struct {
    float left_speed      // speed for left motor (0.0 to 1.0)
    float right_speed     // speed for right motor (0.0 to 1.0)
} ControlOutput_t;

// function declarations

/**
 * @brief kicks off the motor interface module
 * gets things ready to roll
 */
void MotorInterface_Init(void);

/**
 * @brief sets motor speeds based on control output
 * takes the control output and makes the motors move
 * @param[in] output pointer to control output struct
 */
void MotorInterface_SetSpeeds(const ControlOutput_t* output);

/**
 * @brief turns motors on or off
 * flips the switch for the motors
 * @param[in] enable true to turn on false to turn off
 */
void MotorInterface_Enable(bool enable);

/**
 * @brief checks if motors are on
 * sees if the motors are ready to go
 * @return true if on false if off
 */
bool MotorInterface_IsEnabled(void);

/**
 * @brief grabs the current motor status
 * fills in the deets on how the motors are doing
 * @param[out] status pointer to motor status struct
 */
void MotorInterface_GetStatus(MotorStatus_t* status);

/**
 * @brief resets the motor interface
 * wipes everything clean after a mess or restart
 */
void MotorInterface_Reset(void);

/**
 * @brief checks for motor faults
 * looks for any trouble with the motors
 * @return true if there’s a fault false if all good
 */
bool MotorInterface_HasFault(void);

/**
 * @brief sets the speed limit for motors
 * caps how fast the motors can go
 * @param[in] limit speed limit 0 to max speed
 */
void MotorInterface_SetSpeedLimit(uint8_t limit);

/**
 * @brief gets the current motor speed limit
 * tells you the max speed allowed
 * @return current speed limit
 */
uint8_t MotorInterface_GetSpeedLimit(void);

/**
 * @brief sets motor direction
 * decides which way the motors spin
 * @param[in] left_direction true for forward false for backward for left motor
 * @param[in] right_direction true for forward false for backward for right motor
 */
void MotorInterface_SetDirection(bool left_direction, bool right_direction);

/**
 * @brief gets motor direction
 * checks which way the motors are spinning
 * @param[out] left_direction pointer to store left motor direction
 * @param[out] right_direction pointer to store right motor direction
 */
void MotorInterface_GetDirection(bool* left_direction, bool* right_direction);

/**
 * @brief sets motor pwm frequency
 * tweaks how fast the motor signal pulses
 * @param[in] frequency pwm frequency in hz
 */
void MotorInterface_SetPWMFrequency(uint32_t frequency);

/**
 * @brief gets motor pwm frequency
 * tells you the current pulse rate
 * @return current pwm frequency in hz
 */
uint32_t MotorInterface_GetPWMFrequency(void);

/**
 * @brief sets motor accel and decel rates
 * controls how quick the motors speed up or slow down
 * @param[in] acceleration rate in units per second
 * @param[in] deceleration rate in units per second
 */
void MotorInterface_SetAcceleration(float acceleration, float deceleration);

/**
 * @brief gets motor accel and decel rates
 * gives you the current speed up and slow down rates
 * @param[out] acceleration pointer to store accel rate
 * @param[out] deceleration pointer to store decel rate
 */
void MotorInterface_GetAcceleration(float* acceleration, float* deceleration);

/**
 * @brief sets motor control mode like speed or position
 * picks how we control the motors
 * @param[in] mode control mode like speed or position
 */
void MotorInterface_SetControlMode(uint8_t mode);

/**
 * @brief gets current motor control mode
 * tells you how we’re controlling the motors
 * @return current control mode
 */
uint8_t MotorInterface_GetControlMode(void);

/**
 * @brief sets motor fault handling mode like stop or continue
 * decides what to do if there’s a fault
 * @param[in] mode fault handling mode like stop or continue
 */
void MotorInterface_SetFaultHandlingMode(uint8_t mode);

/**
 * @brief gets current motor fault handling mode
 * shows how we handle faults
 * @return current fault handling mode
 */
uint8_t MotorInterface_GetFaultHandlingMode(void);

/**
 * @brief sets motor fault reset command
 * clears faults if we want to
 * @param[in] reset true to reset faults false to ignore
 */
void MotorInterface_SetFaultReset(bool reset);
