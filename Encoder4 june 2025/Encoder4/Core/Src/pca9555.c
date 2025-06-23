/*
 * pca9555.c
 *
 *  Created on: Mar 16, 2025
 *      Author: thebo
 */


#include "pca9555.h"

HAL_StatusTypeDef PCA9555_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    return HAL_I2C_Master_Transmit(hi2c, PCA9555_I2C_ADDRESS, buf, 2, HAL_MAX_DELAY);
}

HAL_StatusTypeDef PCA9555_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data) {
    HAL_StatusTypeDef status;
    status = HAL_I2C_Master_Transmit(hi2c, PCA9555_I2C_ADDRESS, &reg, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;
    return HAL_I2C_Master_Receive(hi2c, PCA9555_I2C_ADDRESS, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef PCA9555_Init(I2C_HandleTypeDef *hi2c) {
    // Set all pins to outputs (0 = output)
    HAL_StatusTypeDef status = HAL_OK;
    status |= PCA9555_WriteReg(hi2c, PCA9555_REG_CONFIG0, 0x00);
    status |= PCA9555_WriteReg(hi2c, PCA9555_REG_CONFIG1, 0x00);
    return status;
}

HAL_StatusTypeDef PCA9555_SetPinDirection(I2C_HandleTypeDef *hi2c, uint8_t pin, uint8_t is_input) {
    uint8_t reg = (pin < 8) ? PCA9555_REG_CONFIG0 : PCA9555_REG_CONFIG1;
    uint8_t bit = pin % 8;
    uint8_t config;

    if (PCA9555_ReadReg(hi2c, reg, &config) != HAL_OK) return HAL_ERROR;
    if (is_input)
        config |= (1 << bit);
    else
        config &= ~(1 << bit);
    return PCA9555_WriteReg(hi2c, reg, config);
}

HAL_StatusTypeDef PCA9555_WritePin(I2C_HandleTypeDef *hi2c, uint8_t pin, uint8_t state) {
    uint8_t reg = (pin < 8) ? PCA9555_REG_OUTPUT0 : PCA9555_REG_OUTPUT1;
    uint8_t bit = pin % 8;
    uint8_t output;

    if (PCA9555_ReadReg(hi2c, reg, &output) != HAL_OK) return HAL_ERROR;
    if (state)
        output |= (1 << bit);
    else
        output &= ~(1 << bit);
    return PCA9555_WriteReg(hi2c, reg, output);
}

HAL_StatusTypeDef PCA9555_ReadPin(I2C_HandleTypeDef *hi2c, uint8_t pin, uint8_t *state) {
    uint8_t reg = (pin < 8) ? PCA9555_REG_INPUT0 : PCA9555_REG_INPUT1;
    uint8_t bit = pin % 8;
    uint8_t input;

    if (PCA9555_ReadReg(hi2c, reg, &input) != HAL_OK) return HAL_ERROR;
    *state = (input >> bit) & 0x01;
    return HAL_OK;
}
