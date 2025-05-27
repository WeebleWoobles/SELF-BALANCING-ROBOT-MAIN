#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "state_estimation.h"
#include "input_management.h"

// Main Control Logic (Balancing and Movement - PID Control)
//
// This module implements the main control logic for the robot.
// It uses the estimated state from the state estimation module to determine
// the necessary motor commands for balancing and movement. The control
// logic employs PID control algorithms to adjust motor speeds based on
// the robot's tilt angle and angular velocity. It also processes user
// inputs (e.g., from a remote control or Bluetooth) to control the robot's
// movement and direction.

// PID Controller Parameters
#define CONTROL_DT                0.01f   // Control loop time step (seconds, 100 Hz)
#define PID_DEFAULT_KP            30.0f   // We need to change this proportional gain
#define PID_DEFAULT_KI            0.0f    // This is default integral gain
#define PID_DEFAULT_KD            1.0f    // default derivative gain
#define PID_OUTPUT_LIMIT          255.0f  // Max duty cycle output

// User Input Structure
typedef struct {
    float forward;      // Forward and backward command (-1.0 to 1.0)
    float turn;         // left and right turn command (-1.0 to 1.0)
    
    bool enable;        // enable for robot
    bool emergency_stop;// emergency stop
} UserInput_t;

// Control Output Structure
typedef struct {
    float motor_left;   // Left motor command (typically 0.0 to 255.0)
    float motor_right;  // Right motor command (typically 0.0 to 255.0)
} ControlOutput_t;

// PID params
typedef struct {
    float Kp;           // proportional gain
    float Ki;           // integral gain
    float Kd;           // derivative gain
    float integral;     // accumulated integral term 
    float prev_error;   // previous error value 
} PIDParams_t;

/**
 * @brief Initialize the control logic module.
 * Sets up initial PID parameters and internal states.
 */
void ControlLogic_Init(void);

/**
 * @brief Update the control logic and compute motor commands.
 * Uses the current state estimate and user input to calculate motor outputs
 * via PID control, respecting enable and emergency stop states.
 * @param[in] state Pointer to the current state estimate from sensors.
 * @param[in] user_input Pointer to the current user input commands.
 * @param[out] output Pointer to the control output structure to fill with motor commands.
 */
void ControlLogic_Update(const StateEstimate_t* state, const UserInput_t* user_input, ControlOutput_t* output);

/**
 * @brief Reset the control logic.
 * Clears internal PID states (e.g., integral, previous error) and prepares the module for a fresh start.
 */
void ControlLogic_Reset(void);

/**
 * @brief Set PID controller parameters.
 * Updates the PID gains used for balancing and movement control.
 * @param[in] kp Proportional gain.
 * @param[in] ki Integral gain.
 * @param[in] kd Derivative gain.
 */
void ControlLogic_SetPID(float kp, float ki, float kd);

/**
 * @brief Get current PID controller parameters.
 * Retrieves the active PID gains for inspection or adjustment.
 * @param[out] kp Pointer to store the proportional gain.
 * @param[out] ki Pointer to store the integral gain.
 * @param[out] kd Pointer to store the derivative gain.
 */
void ControlLogic_GetPID(float* kp, float* ki, float* kd);

/**
 * @brief Set the control output limit.
 * edfines maximum  motor command value like in PWM scaling
 * @param[in] limit output limit value .
 */
void ControlLogic_SetOutputLimit(float limit);

/**
 * @brief get the control output limit
 * rturns the maximum motor command value
 * @return current output limit.
 */
float ControlLogic_GetOutputLimit(void);

/**
 * @brief Enable or disable the control logic.
 * Controls whether the module processes inputs and generates outputs.
 * @param[in] enable True to enable control logic, false to disable.
 */
void ControlLogic_Enable(bool enable);

/**
 * @brief Check if the control logic is enabled.
 * Indicates whether the module is currently active.
 * @return True if enabled, false if disabled.
 */
bool ControlLogic_IsEnabled(void);

/**
 * @brief Set the emergency stop flag.
 * Triggers or clears an emergency stop condition, typically halting motor outputs.
 * @param[in] stop True to activate emergency stop, false to clear it.
 */
void ControlLogic_SetEmergencyStop(bool stop);

/**
 * @brief check for emergency stop
 * tells if module is in emergency stop.
 * @return rrue if emergency stop is active, false if not active
 */
bool ControlLogic_IsEmergencyStop(void);

/**
 * @brief set the user input structure.
 * stores user input for potential use outside of testing
 * @param[in] input pointer to the user input structure to set.
 */
void ControlLogic_SetUserInput(const UserInput_t* input);

/**
 * @brief gets the current user input structure.
 * @param[out] inputs pointer to the user input structure to fill
 */
void ControlLogic_GetUserInput(UserInput_t* input);

/**
 * @brief sets the control loop time step
 * chnges time step in PID calculations 
 * @param[in] change in time step in seconds
 */
void ControlLogic_SetTimeStep(float dt);

/**
 * @brief Get the control loop time step.
 * returns  current time step in control calculations.
 * @return current time step in seconds.
 */
float ControlLogic_GetTimeStep(void);
