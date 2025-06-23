///*
// * mlx90393_mux.h
// *
// *  Created on: Mar 18, 2025
// *      Author: thebo
// */
//
//// mlx90393_mux.h
//#ifndef MLX90393_MUX_H
//#define MLX90393_MUX_H
//
//#include "stm32g0xx_hal.h"
//#include "mlx90393.h"
//#include "pca9555.h"
//
//#define MLX_COUNT 12  // Number of MLX90393 sensors connected via PCA9555 CS lines
//
//// PCA9555 pins assigned to MLX90393 chip select
//// MLX90393 CS lines use PCA9555 pins 0 to (MLX_COUNT - 1)
//#define MLX90393_CS_PIN_BASE 0
//
//// Initializes PCA9555 and all MLX90393 sensors
//HAL_StatusTypeDef MLX90393_MUX_Init(I2C_HandleTypeDef *hi2c);
//
//// Reads from the selected MLX90393 by index
//HAL_StatusTypeDef MLX90393_MUX_Read(int index, int16_t *x, int16_t *y, int16_t *z);
//#endif // MLX90393_MUX_H
