#include "control_logic.h"

// var list
static PIDParams_t pid = {PID_DEFAULT_KP, PID_DEFAULT_KI, PID_DEFAULT_KD, 0, 0}
static float output_limit = PID_OUTPUT_LIMIT
static bool is_enabled = false
static bool emerg_stop = false
static UserInput_t user_input = {0}
static float dt = CONTROL_DT

// starts control logic module
void ControlLogic_Init(void)
{
    pid.Kp = PID_DEFAULT_KP
    pid.Ki = PID_DEFAULT_KI
    pid.Kd = PID_DEFAULT_KD
    pid.integral = 0
    pid.prev_error = 0
    is_enabled = false
    emerg_stop = false
    user_input = (UserInput_t){0}
    dt = CONTROL_DT
}

// updates motor commands with pid 
void ControlLogic_Update(const StateEstimate_t* state, const UserInput_t* input, ControlOutput_t* output)
{
    if (!is_enabled || emerg_stop || state == NULL || input == NULL || output == NULL)
    {
        output->motor_left = 0
        output->motor_right = 0
        return
    }

    // calc error based on tilt angle
    float error = state->tilt_angle
    pid.integral += error * dt
    float deriv = (error - pid.prev_error) / dt
    float pid_out = pid.Kp * error + pid.Ki * pid.integral + pid.Kd * deriv

    // limit pid output
    pid_out = pid_out > output_limit ? output_limit : pid_out < -output_limit ? -output_limit : pid_out

    // mix with user input for movement
    float base_speed = pid_out * input->forward
    float turn_adjust = input->turn * output_limit * 0.5f
    output->motor_left = base_speed - turn_adjust
    output->motor_right = base_speed + turn_adjust

    // clamp outputs
    output->motor_left = output->motor_left > output_limit ? output_limit : output->motor_left < 0 ? 0 : output->motor_left
    output->motor_right = output->motor_right > output_limit ? output_limit : output->motor_right < 0 ? 0 : output->motor_right

    pid.prev_error = error
}

// resets the control logic 
void ControlLogic_Reset(void)
{
    pid.integral = 0
    pid.prev_error = 0
    is_enabled = false
    emerg_stop = false
    user_input = (UserInput_t){0}
}

// tweaks the pid gains
void ControlLogic_SetPID(float kp, float ki, float kd)
{
    pid.Kp = kp
    pid.Ki = ki
    pid.Kd = kd
}

// grabs the pid gains
void ControlLogic_GetPID(float* kp, float* ki, float* kd)
{
    if (kp != NULL) *kp = pid.Kp
    if (ki != NULL) *ki = pid.Ki
    if (kd != NULL) *kd = pid.Kd
}

// sets the max output limit
void ControlLogic_SetOutputLimit(float limit)
{
    output_limit = limit > 0 ? limit : PID_OUTPUT_LIMIT
}

// gets the max output limit
float ControlLogic_GetOutputLimit(void)
{
    return output_limit
}

// turns control logic on or off
void ControlLogic_Enable(bool enable)
{
    is_enabled = enable
}

// checks if control logic is on
bool ControlLogic_IsEnabled(void)
{
    return is_enabled
}

// flips the emerg stop switch
void ControlLogic_SetEmergencyStop(bool stop)
{
    emerg_stop = stop
}

// sees if emerg stop is on
bool ControlLogic_IsEmergencyStop(void)
{
    return emerg_stop
}

// stores user input
void ControlLogic_SetUserInput(const UserInput_t* input)
{
    if (input != NULL) user_input = *input
}

// grabs the user input
void ControlLogic_GetUserInput(UserInput_t* input)
{
    if (input != NULL) *input = user_input
}

// changes the time step for pid
void ControlLogic_SetTimeStep(float dt_in)
{
    dt = dt_in > 0 ? dt_in : CONTROL_DT
}

// gets the current time step
float ControlLogic_GetTimeStep(void)
{
    return dt
}
