#include "control_logic.h"


// Function Implementations

/**
 * @brief Initialize the control logic module.
 */
void ControlLogic_Init(void);

/**
 * @brief Update the control logic and compute motor commands.
 * @param[in] state Pointer to the current state estimate.
 * @param[in] user_input Pointer to the current user input.
 * @param[out] output Pointer to the control output structure to fill.
 */
void ControlLogic_Update(const StateEstimate_t* state, const UserInput_t* user_input, ControlOutput_t* output);

/**
 * @brief Reset the control logic (e.g., after a fault or reset event).
 */
void ControlLogic_Reset(void);

/**
 * @brief Set PID controller parameters.
 * @param[in] kp Proportional gain.
 * @param[in] ki Integral gain.
 * @param[in] kd Derivative gain.
 */
void ControlLogic_SetPID(float kp, float ki, float kd);

/**
 * @brief Get current PID controller parameters.
 * @param[out] kp Pointer to store proportional gain.
 * @param[out] ki Pointer to store integral gain.
 * @param[out] kd Pointer to store derivative gain.
 */
void ControlLogic_GetPID(float* kp, float* ki, float* kd);

/**
 * @brief Set the control output limit.
 * @param[in] limit Output limit (e.g., max PWM value).
 */
void ControlLogic_SetOutputLimit(float limit);

/**
 * @brief Get the control output limit.
 * @return Current output limit.
 */
float ControlLogic_GetOutputLimit(void);

/**
 * @brief Enable or disable the control logic.
 * @param[in] enable True to enable, false to disable.
 */
void ControlLogic_Enable(bool enable);

/**
 * @brief Check if the control logic is enabled.
 * @return True if enabled, false otherwise.
 */
bool ControlLogic_IsEnabled(void);

/**
 * @brief Set the emergency stop flag.
 * @param[in] stop True to set the emergency stop, false to clear it.
 */
void ControlLogic_SetEmergencyStop(bool stop);

/**
 * @brief Check if the emergency stop is active.
 * @return True if emergency stop is active, false otherwise.
 */
bool ControlLogic_IsEmergencyStop(void);

/**
 * @brief Set the user input structure.
 * @param[in] input Pointer to the user input structure to set.
 */
void ControlLogic_SetUserInput(const UserInput_t* input);

/**
 * @brief Get the current user input structure.
 * @param[out] input Pointer to the user input structure to fill.
 */
void ControlLogic_GetUserInput(UserInput_t* input);

/**
 * @brief Set the control loop time step.
 * @param[in] dt Time step in seconds.
 */
void ControlLogic_SetTimeStep(float dt);

/**
 * @brief Get the control loop time step.
 * @return Current time step in seconds.
 */
float ControlLogic_GetTimeStep(void);
