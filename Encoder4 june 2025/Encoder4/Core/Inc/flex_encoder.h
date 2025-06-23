/*
 * flex_encoder.h
 *
 *  Created on: Mar 18, 2025
 *      Author: thebo
 */

// flex_encoder.h
#ifndef FLEX_ENCODER_H
#define FLEX_ENCODER_H

#include "mlx90393.h"

//#define FLEX_ENCODER_COUNT 6  // Number of MLX90393 sensors

// Initializes the sensor multiplexer and sensors
HAL_StatusTypeDef FlexEncoder_Init(I2C_HandleTypeDef *hi2c);

// Updates all sensor readings (X, Y, Z) into the internal data array
HAL_StatusTypeDef FlexEncoder_UpdateAll(void);

// Retrieves the latest (x, y, z) reading for a specific sensor
HAL_StatusTypeDef FlexEncoder_Get(int16_t out[3][MLX_COUNT]);

#endif // FLEX_ENCODER_H
