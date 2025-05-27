#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "state_estimation.h"
#include "input_management.h"

// main control logic (balancing and movement - pid control)
//
// this module implements the main control logic for the robot.
// it uses the estimated state from the state estimation module to determine
// the necessary motor commands for balancing and movement. the control
// logic employs pid control algorithms to adjust motor speeds based on
// the robot's tilt angle and angular velocity. it also processes user
// inputs (e.g., from a remote control or bluetooth) to control the robot's
// movement and direction.

// pid controller parameters
#define CONTROL_DT                0.01f   // control loop time step (seconds, 100 hz)
#define PID_DEFAULT_KP            30.0f   // default proportional gain
#define PID_DEFAULT_KI            0.0f    // default integral gain
#define PID_DEFAULT_KD            1.0f    // default derivative gain
#define PID_OUTPUT_LIMIT          255.0f  // max duty cycle output

// user input structure
typedef struct {
    float forward;      // forward and backward command (-1.0 to 1.0)
    float turn;         // left and right turn command (-1.0 to 1.0)
    bool enable;        // enable for robot
    bool emergency_stop;// emergency stop
} UserInput_t;

// control output structure
typedef struct {
    float motor_left;   // left motor command (typically 0.0 to 255.0)
    float motor_right;  // right motor command (typically 0.0 to 255.0)
} ControlOutput_t;

// pid params
typedef struct {
    float Kp;           // proportional gain
    float Ki;           // integral gain
    float Kd;           // derivative gain
    float integral;     // accumulated integral term 
    float prev_error;   // previous error value 
} PIDParams_t;

// function declarations

/**
 * @brief initialize the control logic module.
 * sets up initial pid parameters and internal states.
 */
void ControlLogic_Init(void);

/**
 * @brief update the control logic and compute motor commands.
 * uses the current state estimate and user input to calculate motor outputs
 * via pid control, respecting enable and emergency stop states.
 * @param[in] state pointer to the current state estimate from sensors.
 * @param[in] user_input pointer to the current user input commands.
 * @param[out] output pointer to the control output structure to fill with motor commands.
 */
void ControlLogic_Update(const StateEstimate_t* state, const UserInput_t* user_input, ControlOutput_t* output);

/**
 * @brief reset the control logic.
 * clears internal pid states (e.g., integral, previous error) and prepares the module for a fresh start.
 */
void ControlLogic_Reset(void);

/**
 * @brief set pid controller parameters.
 * updates the pid gains used for balancing and movement control.
 * @param[in] kp proportional gain.
 * @param[in] ki integral gain.
 * @param[in] kd derivative gain.
 */
void ControlLogic_SetPID(float kp, float ki, float kd);

/**
 * @brief get current pid controller parameters.
 * retrieves the active pid gains for inspection or adjustment.
 * @param[out] kp pointer to store the proportional gain.
 * @param[out] ki pointer to store the integral gain.
 * @param[out] kd pointer to store the derivative gain.
 */
void ControlLogic_GetPID(float* kp, float* ki, float* kd);

/**
 * @brief set the control output limit.
 * defines maximum motor command value like in pwm scaling.
 * @param[in] limit output limit value.
 */
void ControlLogic_SetOutputLimit(float limit);

/**
 * @brief get the control output limit.
 * returns the maximum motor command value.
 * @return current output limit.
 */
float ControlLogic_GetOutputLimit(void);

/**
 * @brief enable or disable the control logic.
 * controls whether the module processes inputs and generates outputs.
 * @param[in] enable true to enable control logic, false to disable.
 */
void ControlLogic_Enable(bool enable);

/**
 * @brief check if the control logic is enabled.
 * indicates whether the module is currently active.
 * @return true if enabled, false if disabled.
 */
bool ControlLogic_IsEnabled(void);

/**
 * @brief set the emergency stop flag.
 * triggers or clears an emergency stop condition, typically halting motor outputs.
 * @param[in] stop true to activate emergency stop, false to clear it.
 */
void ControlLogic_SetEmergencyStop(bool stop);

/**
 * @brief check for emergency stop.
 * tells if module is in emergency stop.
 * @return true if emergency stop is active, false if not active.
 */
bool ControlLogic_IsEmergencyStop(void);

/**
 * @brief set the user input structure.
 * stores user input for potential use outside of testing.
 * @param[in] input pointer to the user input structure to set.
 */
void ControlLogic_SetUserInput(const UserInput_t* input);

/**
 * @brief gets the current user input structure.
 * @param[out] input pointer to the user input structure to fill.
 */
void ControlLogic_GetUserInput(UserInput_t* input);

/**
 * @brief sets the control loop time step.
 * changes time step in pid calculations.
 * @param[in] dt change in time step in seconds.
 */
void ControlLogic_SetTimeStep(float dt);

/**
 * @brief get the control loop time step.
 * returns current time step in control calculations.
 * @return current time step in seconds.
 */
float ControlLogic_GetTimeStep(void);
