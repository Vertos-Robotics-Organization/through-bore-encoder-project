/**
 ******************************************************************************
 * @file           : vertos_calculations.c
 * @brief          : Velocity and acceleration calculations for encoder data
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "vertos_calculations.h"

/* Private variables ---------------------------------------------------------*/

// State variables for velocity and acceleration calculations
static int64_t prevMultiTurnCounts = 0;
static uint32_t prevTimeMillisec = 0;
static float filteredVelocityCPS = 0.0f;
static float filteredAccelCPS2 = 0.0f;
static float lastVelocityCPS = 0.0f;
static int32_t velocityCountsScaled = 0;
static int32_t accelCountsScaled = 0;
static int calculationsReady = 0;

/* Public Functions ----------------------------------------------------------*/

/**
 * @brief Initialize the velocity/acceleration calculation system
 * @retval None
 */
void vertos_calculations_init(void) {
    vertos_calculations_reset();
}

/**
 * @brief Reset all calculation state variables
 * @retval None
 */
void vertos_calculations_reset(void) {
    prevMultiTurnCounts = 0;
    prevTimeMillisec = 0;
    filteredVelocityCPS = 0.0f;
    filteredAccelCPS2 = 0.0f;
    lastVelocityCPS = 0.0f;
    velocityCountsScaled = 0;
    accelCountsScaled = 0;
    calculationsReady = 0;
}

/**
 * @brief Update velocity and acceleration calculations
 * @param multiTurnCounts Current multi-turn encoder counts
 * @param currentTimeMillisec Current system time in milliseconds
 * @retval 1 if calculation successful, 0 if not enough data yet
 */
int vertos_calculations_update(int64_t multiTurnCounts, uint32_t currentTimeMillisec) {
    // Check if we have previous data for calculation
    if (prevTimeMillisec == 0) {
        // First run - store current values and return not ready
        prevMultiTurnCounts = multiTurnCounts;
        prevTimeMillisec = currentTimeMillisec;
        calculationsReady = 0;
        return 0;
    }

    // Calculate time delta
    int64_t deltaCounts = multiTurnCounts - prevMultiTurnCounts;
    uint32_t deltaTime = currentTimeMillisec - prevTimeMillisec;

    // Avoid division by zero
    if (deltaTime == 0) {
        return calculationsReady;
    }

    // Calculate instantaneous velocity in counts per second
    float velocityCPS = (float)deltaCounts * 1000.0f / (float)deltaTime;

    // Get filter coefficients from flash
    float vel_alpha = flash_get_velocity_alpha();
    float acc_alpha = flash_get_accel_alpha();

    // Apply low-pass filter to velocity
    filteredVelocityCPS = vel_alpha * velocityCPS + (1.0f - vel_alpha) * filteredVelocityCPS;

    // Calculate acceleration in counts per second squared
    float accelCPS2 = (filteredVelocityCPS - lastVelocityCPS) * 1000.0f / (float)deltaTime;

    // Apply low-pass filter to acceleration
    filteredAccelCPS2 = acc_alpha * accelCPS2 + (1.0f - acc_alpha) * filteredAccelCPS2;

    // Scale for transmission
    velocityCountsScaled = (int32_t)filteredVelocityCPS;
    accelCountsScaled = (int32_t)filteredAccelCPS2;

    // Update state for next iteration
    lastVelocityCPS = filteredVelocityCPS;
    prevMultiTurnCounts = multiTurnCounts;
    prevTimeMillisec = currentTimeMillisec;

    // Mark as ready after first successful calculation
    calculationsReady = 1;

    return 1;
}

/**
 * @brief Get the current filtered velocity in counts per second
 * @retval Current filtered velocity
 */
float vertos_calculations_get_velocity_cps(void) {
    return filteredVelocityCPS;
}

/**
 * @brief Get the current filtered acceleration in counts per second squared
 * @retval Current filtered acceleration
 */
float vertos_calculations_get_acceleration_cps2(void) {
    return filteredAccelCPS2;
}

/**
 * @brief Get the current scaled velocity for CAN transmission
 * @retval Current scaled velocity
 */
int32_t vertos_calculations_get_velocity_scaled(void) {
    return velocityCountsScaled;
}

/**
 * @brief Get the current scaled acceleration for CAN transmission
 * @retval Current scaled acceleration
 */
int32_t vertos_calculations_get_acceleration_scaled(void) {
    return accelCountsScaled;
}

/**
 * @brief Update filter coefficients from external source
 * @param vel_alpha New velocity filter alpha (0.0 to 1.0)
 * @param acc_alpha New acceleration filter alpha (0.0 to 1.0)
 * @retval 1 if successful, 0 if parameters out of range
 */
int vertos_calculations_set_filter_coefficients(float vel_alpha, float acc_alpha) {
    // Validate ranges
    if (vel_alpha < 0.0f || vel_alpha > 1.0f || acc_alpha < 0.0f || acc_alpha > 1.0f) {
        return 0;
    }

    // Update flash storage
    if (flash_set_velocity_alpha(vel_alpha) != HAL_OK) {
        return 0;
    }
    
    if (flash_set_accel_alpha(acc_alpha) != HAL_OK) {
        return 0;
    }

    return 1;
}

/**
 * @brief Get velocity filter coefficient
 * @retval Current velocity filter alpha
 */
float vertos_calculations_get_velocity_alpha(void) {
    return flash_get_velocity_alpha();
}

/**
 * @brief Get acceleration filter coefficient
 * @retval Current acceleration filter alpha
 */
float vertos_calculations_get_acceleration_alpha(void) {
    return flash_get_accel_alpha();
}

/**
 * @brief Check if calculations are ready (have enough data)
 * @retval 1 if ready, 0 if not
 */
int vertos_calculations_is_ready(void) {
    return calculationsReady;
}