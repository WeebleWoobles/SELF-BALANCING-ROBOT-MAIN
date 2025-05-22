#pragma once

#include <stdint.h>
#include <stdbool.h>

// Input Management Module (Bluetooth Xbox Controller Interface)
//
// This module will handle the input from the Xbox controller.
// It will interface with the controller hardware and implement
// functions for reading button presses, joystick movements, and other
// inputs. The module will convert the raw input data into usable
// formats and provide it to the main control logic. It will also
// handle any necessary debouncing or filtering of the input signals.
//
//  HAL/Scheduler


// Input Management Parameters
#define INPUT_JOYSTICK_DEADZONE   0.05f   // Deadzone for joystick input
#define INPUT_UPDATE_PERIOD_MS    10      // Input polling period in ms

// Xbox Controller Button Definitions (example bitmasks)
#define BUTTON_A      (1 << 0)
#define BUTTON_B      (1 << 1)
#define BUTTON_X      (1 << 2)
#define BUTTON_Y      (1 << 3)
#define BUTTON_LB     (1 << 4)
#define BUTTON_RB     (1 << 5)
#define BUTTON_BACK   (1 << 6)
#define BUTTON_START  (1 << 7)

// Joystick Axes Structure
typedef struct {
    float left_x;   // Left joystick X-axis (-1.0 to 1.0)
    float left_y;   // Left joystick Y-axis (-1.0 to 1.0)
    float right_x;  // Right joystick X-axis (-1.0 to 1.0)
    float right_y;  // Right joystick Y-axis (-1.0 to 1.0)
} JoystickAxes_t;

// Controller Input Structure
typedef struct {
    JoystickAxes_t joysticks;
    uint16_t buttons;      // Bitmask for button states
    bool connected;        // Controller connection status
} ControllerInput_t;

// Function Declarations

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