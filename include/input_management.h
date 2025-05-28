#pragma once

#include <stdint.h>
#include <stdbool.h>

// input management (bluetooth xbox controller interface)
//
// hey, this module handles inputs from an xbox controller via bluetooth.
// it reads button presses, joystick moves, and other goodies, turning raw
// data into something useful for the main control stuff. it also deals
// with debouncing and filtering to keep things smooth.

#define INPUT_JOYSTICK_DEADZONE   0.05f   // deadzone for joystick inputs
#define INPUT_UPDATE_PERIOD_MS    10      // how often we check inputs in ms

// xbox controller button definitions (bitmasks for fun)
#define BUTTON_A      (1 << 0)
#define BUTTON_B      (1 << 1)
#define BUTTON_X      (1 << 2)
#define BUTTON_Y      (1 << 3)
#define BUTTON_LB     (1 << 4)
#define BUTTON_RB     (1 << 5)
#define BUTTON_BACK   (1 << 6)
#define BUTTON_START  (1 << 7)

// joystick axes structure
typedef struct {
    float left_x;   // left joystick x-axis (-1.0 to 1.0)
    float left_y;   // left joystick y-axis (-1.0 to 1.0)
    float right_x;  // right joystick x-axis (-1.0 to 1.0)
    float right_y;  // right joystick y-axis (-1.0 to 1.0)
} JoystickAxes_t;

// controller input structure
typedef struct {
    JoystickAxes_t joysticks;
    uint16_t buttons;      // bitmask for button states
    bool connected;        // is the controller plugged in?
} ControllerInput_t;

// function declarations

/**
 * @brief kicks off the input management module.
 * gets everything ready to roll.
 */
void InputManagement_Init(void);

/**
 * @brief polls and updates the controller state.
 * grabs the latest input data and tweaks it.
 * @param[out] input pointer to the controller input struct to fill up.
 * @return true if it worked, false if it flopped.
 */
bool InputManagement_Update(ControllerInput_t* input);

/**
 * @brief checks if the controller is connected.
 * quick peek to see if we’re linked up.
 * @return true if connected, false if not.
 */
bool InputManagement_IsConnected(void);

/**
 * @brief grabs the latest controller input.
 * hands over the current input goodies.
 * @param[out] input pointer to the controller input struct to fill.
 */
void InputManagement_GetInput(ControllerInput_t* input);

/**
 * @brief sets the joystick deadzone.
 * adjusts how much wiggle we ignore on the sticks.
 * @param[in] deadzone deadzone value (0.0 to 1.0).
 */
void InputManagement_SetDeadzone(float deadzone);

/**
 * @brief gets the current joystick deadzone.
 * tells you how much stick movement we skip.
 * @return deadzone value.
 */
float InputManagement_GetDeadzone(void);

/**
 * @brief checks if a button is pressed.
 * sees if a specific button is getting mashed.
 * @param[in] input pointer to the controller input struct.
 * @param[in] button bitmask to check.
 * @return true if pressed, false if not.
 */
bool InputManagement_IsButtonPressed(const ControllerInput_t* input, uint16_t button);

/**
 * @brief gets the joystick axes values.
 * pulls the latest stick positions.
 * @param[in] input pointer to the controller input struct.
 * @param[out] axes pointer to the joystick axes struct to fill.
 */
void InputManagement_GetJoystickAxes(const ControllerInput_t* input, JoystickAxes_t* axes);

/**
 * @brief sets the controller connection status.
 * flips the connected switch.
 * @param[in] connected true if connected, false if disconnected.
 */
void InputManagement_SetConnectionStatus(bool connected);

/**
 * @brief gets the controller connection status.
 * checks if we’re still chatting with the controller.
 * @return true if connected, false if not.
 */
bool InputManagement_GetConnectionStatus(void);

/**
 * @brief resets the input management module.
 * wipes the slate clean after a mess or restart.
 */
void InputManagement_Reset(void);
