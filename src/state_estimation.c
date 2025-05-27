#include "state_estimation.h"
#include <math.h>

// vars 
static StateEstimate_t current_state = {0}
static float filter_k = COMPLEMENTARY_FILTER_K
static float dt = STATE_ESTIMATION_DT
static bool is_initialized = false

// starts the state estimation module
void StateEstimation_Init(void)
{
    if (!is_initialized)
    {
        current_state.tilt_angle = 0
        current_state.angular_velocity = 0
        current_state.accel_angle = 0
        current_state.gyro_rate = 0
        is_initialized = true
    }
}

// updates the state estimate with imu data
void StateEstimation_Update(const IMU_Data_t* imu_data, StateEstimate_t* state)
{
    if (!is_initialized || imu_data == NULL || state == NULL)
    {
        return
    }

    // figure out accel angle (pitch) from accel_x and accel_z
    float accel_angle = atan2(imu_data->accel_x, imu_data->accel_z) * 180 / M_PI

    // grab gyro rate (y-axis for pitch)
    float gyro_rate = imu_data->gyro_y

    // save the raw data
    current_state.accel_angle = accel_angle
    current_state.gyro_rate = gyro_rate

    // mix em with complimentary filter
    // tilt = k * (old tilt + gyro rate * dt) + (1-k) * accel angle
    current_state.tilt_angle = filter_k * (current_state.tilt_angle + gyro_rate * dt) + (1 - filter_k) * accel_angle

    // angular velocity straight from gyro
    current_state.angular_velocity = gyro_rate

    // pass the updated state back
    *state = current_state
}

// wipes the state estimator clean
void StateEstimation_Reset(void)
{
    if (is_initialized)
    {
        current_state = (StateEstimate_t){0}
        filter_k = COMPLEMENTARY_FILTER_K
        dt = STATE_ESTIMATION_DT
        is_initialized = false
    }
}

// grabs the current tilt angle estimate
float StateEstimation_GetTiltAngle(void)
{
    return current_state.tilt_angle
}

// grabs the current angular velocity estimate
float StateEstimation_GetAngularVelocity(void)
{
    return current_state.angular_velocity
}

// grabs the current accel angle estimate
float StateEstimation_GetAccelAngle(void)
{
    return current_state.accel_angle
}

// grabs the current gyro rate estimate
float StateEstimation_GetGyroRate(void)
{
    return current_state.gyro_rate
}

// gets the full state estimate
void StateEstimation_GetStateEstimate(StateEstimate_t* state)
{
    if (state != NULL)
    {
        *state = current_state
    }
}

// sets the complementary filter coefficient
void StateEstimation_SetComplementaryFilterK(float k)
{
    if (k >= 0 && k <= 1)
    {
        filter_k = k
    }
}

// gets the complementary filter coefficient
float StateEstimation_GetComplementaryFilterK(void)
{
    return filter_k
}

// sets the time step for state estimation
void StateEstimation_SetTimeStep(float dt)
{
    if (dt > 0)
    {
        dt = dt
    }
}

// gets the time step for state estimation
float StateEstimation_GetTimeStep(void)
{
    return dt
}
