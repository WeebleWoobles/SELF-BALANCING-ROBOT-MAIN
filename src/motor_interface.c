#include "motor_interface.h"
#include <stdio.h>
#include <stdlib.h> // for rand()

// motor state variables (actual values need to be entered)
static MotorStatus_t current_status = {0.0f, 0.0f, false, false, false};
static uint8_t speed_limit = MOTOR_MAX_SPEED;
static bool left_direction = true;  // true = forward, false = reverse
static bool right_direction = true;
static uint32_t pwm_frequency = 1000; // default PWM frequency
static float acceleration = 50.0f;    // default acceleration
static float deceleration = 50.0f;    // default deceleration
static uint8_t control_mode = 0;      // speed mode
static uint8_t fault_handling_mode = 0; // stop mode
static bool is_initialized = false;

// mock write function, need to put real function calls here
static void mock_set_motor_speed(uint8_t motor, float speed) {
    if (motor == 0) { // left motor
        current_status.speed_left = (speed > speed_limit) ? speed_limit : speed;
    } else if (motor == 1) { // right motor
        current_status.speed_right = (speed > speed_limit) ? speed_limit : speed;
    }
}

static void mock_set_motor_direction(uint8_t motor, bool direction) {
    if (motor == 0) { // left motor
        left_direction = direction;
    } else if (motor == 1) { // right motor
        right_direction = direction;
    }
}

static void mock_set_pwm_frequency(uint32_t frequency) {
    pwm_frequency = frequency; // set pwm frequency
}

static void mock_check_fault(uint8_t motor) {
    // fault detection simulation
    if (rand() % 100 < 5) { // chance of fault is 5%
        if (motor == 0) current_status.fault_left = true;
        else if (motor == 1) current_status.fault_right = true;
    }
}

/**
 * @brief motor interface module
 */
void MotorInterface_Init(void) {
    if (!is_initialized) {
        // hardware motor driver mock setup since we don't have it wired yet
        current_status.enabled = false;
        is_initialized = true;
        printf("Motor Interface is init\n");
    }
}

/**
 * @brief motor speeds based on our control output
 * @param[in] output pointer to ControlOutput_t struct
 */
void MotorInterface_SetSpeeds(const ControlOutput_t* output) {
    if (!is_initialized || !current_status.enabled || output == NULL) {
        return;
    }

    // apply speeds from control output assuming normalized 0.0 to 1.0
    float left_speed = output->motor_left;
    float right_speed = output->motor_right;
    if (left_speed < 0.0f || left_speed > 1.0f || right_speed < 0.0f || right_speed > 1.0f) {
        printf("Invalid speed values, must be between 0.0 and 1.0\n");
        return;
    }

    // Clamp to [0,1] before scaling
    if (left_speed < 0.0f) left_speed = 0.0f;
    if (left_speed > 1.0f) left_speed = 1.0f;
    if (right_speed < 0.0f) right_speed = 0.0f;
    if (right_speed > 1.0f) right_speed = 1.0f;

    mock_set_motor_speed(0, left_speed * speed_limit);  // left motor
    mock_set_motor_speed(1, right_speed * speed_limit); // right motor

    // check for faults after setting speeds
    mock_check_fault(0);
    mock_check_fault(1);
}

/**
 * @brief motor enable/disable
 * @param[in] enable True to enable, false to disable
 */
void MotorInterface_Enable(bool enable) {
    if (is_initialized) {
        current_status.enabled = enable;
        if (!enable) {
            current_status.speed_left = 0.0f;
            current_status.speed_right = 0.0f;
            mock_set_motor_speed(0, 0.0f);
            mock_set_motor_speed(1, 0.0f);
        }
    }
}

/**
 * @brief Check if the motors are set
 * @return True if enabled, false disabled
 */
bool MotorInterface_IsEnabled(void) {
    return current_status.enabled;
}

/**
 * @brief Gets motor status
 * @param[out] status pointer to MotorStatus_t struct
 */
void MotorInterface_GetStatus(MotorStatus_t* status) {
    if (status != NULL) {
        *status = current_status;
    }
}

/**
 * @brief resets motor interface
 */
void MotorInterface_Reset(void) {
    if (is_initialized) {
        current_status = (MotorStatus_t){0.0f, 0.0f, false, false, false};
        speed_limit = MOTOR_MAX_SPEED;
        left_direction = true;
        right_direction = true;
        pwm_frequency = 1000;
        acceleration = 50.0f;
        deceleration = 50.0f;
        control_mode = 0;
        fault_handling_mode = 0;
        is_initialized = false;
        printf("Motor Interface has been reset\n");
    }
}

/**
 * @brief Checks if there's motor faults
 * @return True if a fault is detected, false if no fault is detected
 */
bool MotorInterface_HasFault(void) {
    return current_status.fault_left || current_status.fault_right;
}

/**
 * @brief Set the speed limit for motor
 * @param[in] limit Speed limit 0 to MOTOR_MAX_SPEED
 */
void MotorInterface_SetSpeedLimit(uint8_t limit) {
    if (limit <= MOTOR_MAX_SPEED) {
        speed_limit = limit;
    }
}

/**
 * @brief says current motor speed limit
 * @return current speed limit
 */
uint8_t MotorInterface_GetSpeedLimit(void) {
    return speed_limit;
}

/**
 * @brief sets motor direction
 * @param[in] left_dir Direction for left motor true for forward, false for backwards
 * @param[in] right_dir Direction for right motor true for forward, false for backwards
 */
void MotorInterface_SetDirection(bool left_dir, bool right_dir) {
    mock_set_motor_direction(0, left_dir);
    mock_set_motor_direction(1, right_dir);
}

/**
 * @brief gets motor direction
 * @param[out] left_dir pointer to store left motor direction
 * @param[out] right_dir pointer to store right motor direction
 */
void MotorInterface_GetDirection(bool* left_dir, bool* right_dir) {
    if (left_dir != NULL) *left_dir = left_direction;
    if (right_dir != NULL) *right_dir = right_direction;
}

/**
 * @brief motor pwm freq
 * @param[in] frequency pwm frequency in Hz.
 */
void MotorInterface_SetPWMFrequency(uint32_t frequency) {
    if (frequency > 0) {
        mock_set_pwm_frequency(frequency);
    }
}

/**
 * @brief gets motor pwm frequency
 * @return current pwm frequency in Hz
 */
uint32_t MotorInterface_GetPWMFrequency(void) {
    return pwm_frequency;
}

/**
 * @brief motor accel/decel
 * @param[in] accel acceleration rate units per second
 * @param[in] decel deceleration rate units per second
 */
void MotorInterface_SetAcceleration(float accel, float decel) {
    if (accel >= 0.0f) acceleration = accel;
    if (decel >= 0.0f) deceleration = decel;
}

/**
 * @brief get the current motor acceleration and deceleration rates
 * @param[out] accel pointer to store acceleration rate
 * @param[out] decel pointer to store deceleration rate
 */
void MotorInterface_GetAcceleration(float* accel, float* decel) {
    if (accel != NULL) *accel = acceleration;
    if (decel != NULL) *decel = deceleration;
}

/**
 * @brief motor control mode (speed, position)
 * @param[in] mode control mode (speed, position).
 */
void MotorInterface_SetControlMode(uint8_t mode) {
    control_mode = mode;
}

/**
 * @brief gets current motor control mode
 * @return current control mode
 */
uint8_t MotorInterface_GetControlMode(void) {
    return control_mode;
}

/**
 * @brief set the motor fault handling mode like stop or continue 
 * @param[in] mode fault handling mode like stop and continue
 */
void MotorInterface_SetFaultHandlingMode(uint8_t mode) {
    fault_handling_mode = mode;
}

/**
 * @brief gets current motor fault handling mode
 * @return current fault handling mode
 */
uint8_t MotorInterface_GetFaultHandlingMode(void) {
    return fault_handling_mode;
}

/**
 * @brief sets motor fault reset command
 * @param[in] reset true to reset faults, false to ignore faults
 */
void MotorInterface_SetFaultReset(bool reset) {
    if (reset && is_initialized) {
        current_status.fault_left = false;
        current_status.fault_right = false;
        printf("Motor faults reset\n");
    }
}
