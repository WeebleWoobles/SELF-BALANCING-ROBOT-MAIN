#include "input_management.h"


// Function Implementations

/**
 * @brief Initialize the input management module.
 */
void InputManagement_Init(void);

/**
 * @brief Poll and update the controller input state.
 * @param[out] input Pointer to ControllerInput_t structure to fill.
 * @return true if input was updated successfully, false otherwise.
 */
bool InputManagement_Update(ControllerInput_t* input);

/**
 * @brief Check if the controller is connected.
 * @return true if connected, false otherwise.
 */
bool InputManagement_IsConnected(void);

/**
 * @brief Get the latest controller input.
 * @param[out] input Pointer to ControllerInput_t structure to fill.
 */
void InputManagement_GetInput(ControllerInput_t* input);

/**
 * @brief Set the joystick deadzone.
 * @param[in] deadzone Deadzone value (0.0 to 1.0).
 */
void InputManagement_SetDeadzone(float deadzone);

/**
 * @brief Get the current joystick deadzone.
 * @return Deadzone value.
 */
float InputManagement_GetDeadzone(void);

/**
 * @brief Check if a specific button is pressed.
 * @param[in] input Pointer to ControllerInput_t structure.
 * @param[in] button Button bitmask to check.
 * @return true if the button is pressed, false otherwise.
 */
bool InputManagement_IsButtonPressed(const ControllerInput_t* input, uint16_t button);

/**
 * @brief Get the joystick axes values.
 * @param[in] input Pointer to ControllerInput_t structure.
 * @param[out] axes Pointer to JoystickAxes_t structure to fill.
 */
void InputManagement_GetJoystickAxes(const ControllerInput_t* input, JoystickAxes_t* axes);

/**
 * @brief Set the controller connection status.
 * @param[in] connected True if connected, false otherwise.
 */
void InputManagement_SetConnectionStatus(bool connected);

/**
 * @brief Get the controller connection status.
 * @return true if connected, false otherwise.
 */
bool InputManagement_GetConnectionStatus(void);

/**
 * @brief Reset the input management module (e.g., after a fault or reset event).
 */
void InputManagement_Reset(void);