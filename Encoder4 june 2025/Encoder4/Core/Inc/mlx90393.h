/*
 * mlx90393.h
 *
 *  Created on: Mar 18, 2025
 *      Author: thebo
 */

// mlx90393.h
#ifndef MLX90393_H
#define MLX90393_H

#include "stm32g0xx_hal.h"

// SPI Handle
extern SPI_HandleTypeDef hspi2;

// MLX90393 Commands
#define MLX_CMD_SB  0x10  // Start Burst Mode
#define MLX_CMD_SW  0x20  // Start Wake-on-Change
#define MLX_CMD_SM  0x30  // Start Single Measurement
#define MLX_CMD_RM  0x40  // Read Measurement
#define MLX_CMD_RR  0x50  // Read Register
#define MLX_CMD_WR  0x60  // Write Register
#define MLX_CMD_EX  0x80  // Exit Mode
#define MLX_CMD_RT  0xF0  // Reset
#define MLX_CMD_HR  0xD0  // Recall Memory
#define MLX_CMD_HS  0xE0  // Store Memory

// Axis selection bits
#define AXIS_X 0x02
#define AXIS_Y 0x04
#define AXIS_Z 0x08
#define AXIS_T 0x01

#define MLX_COUNT 6  // Number of MLX90393 sensors connected via PCA9555 CS lines

// PCA9555 pins assigned to MLX90393 chip select
// MLX90393 CS lines use PCA9555 pins 0 to (MLX_COUNT - 1)
#define MLX90393_CS_PIN_BASE 0


// Public API
HAL_StatusTypeDef MLX90393_MUX_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MLX90393_Reset(int index);
HAL_StatusTypeDef MLX90393_ConfigureDefault(int index);
HAL_StatusTypeDef MLX90393_ReadMeasurement(int index, int16_t *x, int16_t *y, int16_t *z);
//HAL_StatusTypeDef MLX90393_ReadRegister(uint8_t reg, uint16_t *value);
//HAL_StatusTypeDef MLX90393_WriteRegister(uint8_t reg, uint16_t value);

#endif // MLX90393_H

