#include "state_estimation.h"

// Function Implementations

/**
 * @brief Initialize the state estimation module.
 */
void StateEstimation_Init(void);

/**
 * @brief Update the state estimate using IMU data.
 * @param[in] imu_data Pointer to the latest IMU data.
 * @param[out] state Pointer to the state estimate structure to update.
 */
void StateEstimation_Update(const IMU_Data_t* imu_data, StateEstimate_t* state);

/**
 * @brief Reset the state estimator (e.g., after a fault or reset event).
 */
void StateEstimation_Reset(void);

/**
 * @brief Get the current tilt angle estimate.
 * @return Current tilt angle.
 */
float StateEstimation_GetTiltAngle(void);

/**
 * @brief Get the current angular velocity estimate.
 * @return Current angular velocity.
 */
float StateEstimation_GetAngularVelocity(void);

/**
 * @brief Get the current accelerometer angle estimate.
 * @return Current accelerometer angle.
 */
float StateEstimation_GetAccelAngle(void);

/**
 * @brief Get the current gyroscope rate estimate.
 * @return Current gyroscope rate.
 */
float StateEstimation_GetGyroRate(void);

/**
 * @brief Get the current state estimate.
 * @param[out] state Pointer to the state estimate structure to fill.
 */
void StateEstimation_GetStateEstimate(StateEstimate_t* state);

/**
 * @brief Set the complementary filter coefficient.
 * @param[in] k Complementary filter coefficient (0 < k < 1).
 */
void StateEstimation_SetComplementaryFilterK(float k);

/**
 * @brief Get the complementary filter coefficient.
 * @return Current complementary filter coefficient.
 */
float StateEstimation_GetComplementaryFilterK(void);

/**
 * @brief Set the time step for state estimation.
 * @param[in] dt Time step in seconds.
 */
void StateEstimation_SetTimeStep(float dt);

/**
 * @brief Get the time step for state estimation.
 * @return Current time step in seconds.
 */
float StateEstimation_GetTimeStep(void);

