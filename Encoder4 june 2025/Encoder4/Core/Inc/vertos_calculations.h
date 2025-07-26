/**
 ******************************************************************************
 * @file           : vertos_calculations.h
 * @brief          : Header for velocity and acceleration calculations
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

#ifndef __VERTOS_CALCULATIONS_H
#define __VERTOS_CALCULATIONS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "flash_config.h"

/* Exported constants --------------------------------------------------------*/
#define VELOCITY_SCALE_FACTOR 1000.0f
#define ACCELERATION_SCALE_FACTOR 100.0f

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief Initialize the velocity/acceleration calculation system
 * @retval None
 */
void vertos_calculations_init(void);

/**
 * @brief Reset all calculation state variables
 * @retval None
 */
void vertos_calculations_reset(void);

/**
 * @brief Update velocity and acceleration calculations
 * @param multiTurnCounts Current multi-turn encoder counts
 * @param currentTimeMillisec Current system time in milliseconds
 * @retval 1 if calculation successful, 0 if not enough data yet
 */
int vertos_calculations_update(int64_t multiTurnCounts, uint32_t currentTimeMillisec);

/**
 * @brief Get the current filtered velocity in counts per second
 * @retval Current filtered velocity
 */
float vertos_calculations_get_velocity_cps(void);

/**
 * @brief Get the current filtered acceleration in counts per second squared
 * @retval Current filtered acceleration
 */
float vertos_calculations_get_acceleration_cps2(void);

/**
 * @brief Get the current scaled velocity for CAN transmission (32-bit signed)
 * @retval Current scaled velocity
 */
int32_t vertos_calculations_get_velocity_scaled(void);

/**
 * @brief Get the current scaled acceleration for CAN transmission (32-bit signed)
 * @retval Current scaled acceleration
 */
int32_t vertos_calculations_get_acceleration_scaled(void);

/**
 * @brief Update filter coefficients from external source
 * @param vel_alpha New velocity filter alpha (0.0 to 1.0)
 * @param acc_alpha New acceleration filter alpha (0.0 to 1.0)
 * @retval 1 if successful, 0 if parameters out of range
 */
int vertos_calculations_set_filter_coefficients(float vel_alpha, float acc_alpha);

/**
 * @brief Get velocity filter coefficient
 * @retval Current velocity filter alpha
 */
float vertos_calculations_get_velocity_alpha(void);

/**
 * @brief Get acceleration filter coefficient
 * @retval Current acceleration filter alpha
 */
float vertos_calculations_get_acceleration_alpha(void);

/**
 * @brief Check if calculations are ready (have enough data)
 * @retval 1 if ready, 0 if not
 */
int vertos_calculations_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* __VERTOS_CALCULATIONS_H */