#pragma once

#include "input_management.h"
#include <stdio.h>

// inputs for testing
static ControllerInput_t current_input = {0};
static float joystick_deadzone = INPUT_JOYSTICK_DEADZONE;
static bool is_initialized = false;
static bool connection_status = false;

// controller input data
static void mock_read_input(ControllerInput_t* input) {
    // this is where we put our deadzone information
    input->joysticks.left_x = 0.1f;  // replace with actual values
    input->joysticks.left_y = 0.8f;  
    input->joysticks.right_x = -0.9f;
    input->joysticks.right_y = 0.0f;
    input->buttons = BUTTON_A | BUTTON_B; //testing a and b buttons
    input->connected = connection_status;
}

// deadzone values
static void apply_deadzone(JoystickAxes_t* axes) {
    if (axes->left_x >= -joystick_deadzone && axes->left_x <= joystick_deadzone) {
        axes->left_x = 0.0f;

    }
    if (axes->left_y >= -joystick_deadzone && axes->left_y <= joystick_deadzone) {
        axes->left_y = 0.0f;


    }
    
    if (axes->right_x >= -joystick_deadzone && axes->right_x <= joystick_deadzone) {
        axes->right_x = 0.0f;
    }
   
   
    if (axes->right_y >= -joystick_deadzone && axes->right_y <= joystick_deadzone) {
        axes->right_y = 0.0f;
    }
}

/**
 * @brief input management
 */
void InputManagement_Init(void) {
    if (!is_initialized) {
        // bluetooth connection
        connection_status = true; // this runs the connetion when initialized
        is_initialized = true;
        current_input.connected = true;
        printf("Input management achieved chaaa\n");
    }
}

/**
 * @brief usies polling to update controller state
 * @param[out] input pointer to ControllerInput_t struct
 * @return true if inputs updated, false if not
 */
bool InputManagement_Update(ControllerInput_t* input) {
    if (!is_initialized || !connection_status || input == NULL) {
        return false;
    }

    // hardware simaulation
    mock_read_input(&current_input);

    // joystick value deadzones
    apply_deadzone(&current_input.joysticks);

    // updated input to structure
    *input = current_input;
    return true;
}

/**
 * @brief is controller connected?
 * @return true or false
 */
bool InputManagement_IsConnected(void) {
    return connection_status;
}

/**
 * @brief gets controller input
 * @param[out] input pointer to the ControllerInput_t struct
 */
void InputManagement_GetInput(ControllerInput_t* input) {
    if (input != NULL) {
        *input = current_input;
    }
}

/**
 * @brief joystick deadzone
 * @param[in] deadzone deadzone values from  0.0 to 1.0
 */
void InputManagement_SetDeadzone(float deadzone) {
    if (deadzone >= 0.0f && deadzone <= 1.0f) {
        joystick_deadzone = deadzone;
    }
}

/**
 * @brief gets deadzone values
 * @return deadzone values
 */
float InputManagement_GetDeadzone(void) {
    return joystick_deadzone;
}

/**
 * @brief check if a button is pressed
 * @param[in] input pointer to ControllerInput_t struct
 * @param[in] button bitask
 * @return if button pressed, true, false otherwise
 */
bool InputManagement_IsButtonPressed(const ControllerInput_t* input, uint16_t button) {
    if (input == NULL) {
        return false;
    }
    return (input->buttons & button) != 0;
}

/**
 * @brief joystick axis values
 * @param[in] input pointer to ControllerInput_t struct
 * @param[out] axes pointer to JoystickAxes_t struct
 */
void InputManagement_GetJoystickAxes(const ControllerInput_t* input, JoystickAxes_t* axes) {
    if (input != NULL && axes != NULL) {
        *axes = input->joysticks;
    }
}

/**
 * @brief controller connection status
 * @param[in] connected True if connected, false disconnected
 */
void InputManagement_SetConnectionStatus(bool connected) {
    connection_status = connected;
    current_input.connected = connected;
}

/**
 * @brief controller connected
 * @return true if connected, false if disconnected
 */
bool InputManagement_GetConnectionStatus(void) {
    return InputManagement_IsConnected();
}

/**
 * @brief reset the input management module after reset or fault
 */
void InputManagement_Reset(void) {
    if (is_initialized) {
        current_input = (ControllerInput_t){0};
        connection_status = false;
        is_initialized = false;
        printf("Initializing reset for input management\n");
    }
}
