#include "motor_interface.h"
#include <stdio.h> 

// mototr state variables(actual values need to be entered)
static MotorStatus_t current_status = {0.0f, 0.0f, false, false, false};
static uint8_t speed_limit = MOTOR_MAX_SPEED;
static bool left_direction = true;  // true = forward false = reverse
static bool right_direction = true;
static uint32_t pwm_frequency = 1000; // default PWM frequency
static float acceleration = 50.0f;    // default acceleration
static float deceleration = 50.0f;    // deceleration
static uint8_t control_mode = 0;      // speed mode
static uint8_t fault_handling_mode = 0; // stop mode
static bool is_initialized = false;

// mock wrte function, need to put real function calls here
static void mock_set_motor_speed(uint8_t motor, float speed) {
    if (motor == 0) { // left motor
        current_status.speed_left = (speed > speed_limit) ? speed_limit : speed;
    } else if (motor == 1) { // rite motor
        current_status.speed_right = (speed > speed_limit) ? speed_limit : speed;
    }
}

static void mock_set_motor_direction(uint8_t motor, bool direction) {
    if (motor == 0) { // left motor
        left_direction = direction;
    } else if (motor == 1) { // rite motor
        right_direction = direction;
    }
}

static void mock_set_pwm_frequency(uint32_t frequency) {
    pwm_frequency = frequency; // set pwm frequency
}

static void mock_check_fault(uint8_t motor) {
    // fault detection simualtion
    if (rand() % 100 < 5) { // chnace of fault is 5%
        if (motor == 0) current_status.fault_left = true;
        else if (motor == 1) current_status.fault_right = true;
    }
}

/**
 * @brief motor interface module
 */
void MotorInterface_Init(void) {
    if (!is_initialized) {
        // hardware motor driver mock setup since we dont have it wired yet
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
    float left_speed = output->left_speed * MOTOR_MAX_SPEED;
    float right_speed = output->right_speed * MOTOR_MAX_SPEED;

    mock_set_motor_speed(0, left_speed);  // left motor
    mock_set_motor_speed(1, right_speed); // rite motor

    // check for faults after setting speeds
    mock_check_fault(0);
    mock_check_fault(1);
}

/**
 * @brief motor enable/disable
 * @param[in] True to enable, false to disable
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
 * @brief Gets motor tatus
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
 * @brief Checks if theres motor faults
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
 * @param[in] left_direction Direction for left motor true for forward, false for backwards
 * @param[in] right_direction Direction for rite motor true for forward, false for backwards
 */
void MotorInterface_SetDirection(bool left_direction, bool right_direction) {
    mock_set_motor_direction(0, left_direction);
    mock_set_motor_direction(1, right_direction);
}

/**
 * @brief gets motor direction
 * @param[out] left_direction pointer to store left motor direction
 * @param[out] right_direction pointer to store rite motor direction
 */
void MotorInterface_GetDirection(bool* left_direction, bool* right_direction) {
    if (left_direction != NULL) *left_direction = left_direction;
    if (right_direction != NULL) *right_direction = right_direction;
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
 * @param[in] acceleration acceleration rate units per second
 * @param[in] deceleration deceleration rate units per second
 */
void MotorInterface_SetAcceleration(float acceleration, float deceleration) {
    if (acceleration >= 0.0f) this->acceleration = acceleration;
    if (deceleration >= 0.0f) this->deceleration = deceleration;
}

/**
 * @brief get the current motor acceleration and deceleration rates
 * @param[out] acceleration pointer to store acceleration rate
 * @param[out] deceleration pointer to store deceleration rate
 */
void MotorInterface_GetAcceleration(float* acceleration, float* deceleration) {
    if (acceleration != NULL) *acceleration = this->acceleration;
    if (deceleration != NULL) *deceleration = this->deceleration;
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
 * @brief set the motor fault handling mode like stop or contirnue 
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
