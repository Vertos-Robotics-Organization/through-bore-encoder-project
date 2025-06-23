/*
 * pca9555.h
 *
 *  Created on: Mar 16, 2025
 *      Author: thebo
 */

#ifndef PCA9555_H
#define PCA9555_H

#include "stm32g0xx_hal.h"

// Register addresses
#define PCA9555_REG_INPUT0      0x00
#define PCA9555_REG_INPUT1      0x01
#define PCA9555_REG_OUTPUT0     0x02
#define PCA9555_REG_OUTPUT1     0x03
#define PCA9555_REG_CONFIG0     0x06
#define PCA9555_REG_CONFIG1     0x07

// Default address: 0b0100_A2A1A0 (set your jumpers accordingly)
// For A2,A1,A0 = 0,0,0 → 0x40
#define PCA9555_I2C_ADDRESS     (0x20 << 1)  // Shifted left for HAL

HAL_StatusTypeDef PCA9555_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t data);
HAL_StatusTypeDef PCA9555_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef PCA9555_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef PCA9555_SetPinDirection(I2C_HandleTypeDef *hi2c, uint8_t pin, uint8_t is_input);
HAL_StatusTypeDef PCA9555_WritePin(I2C_HandleTypeDef *hi2c, uint8_t pin, uint8_t state);
HAL_StatusTypeDef PCA9555_ReadPin(I2C_HandleTypeDef *hi2c, uint8_t pin, uint8_t *state);

#endif
