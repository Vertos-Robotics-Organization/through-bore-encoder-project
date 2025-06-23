/*
 * flex_encoder.c
 *
 *  Created on: Mar 18, 2025
 *      Author: thebo
 */

#include "flex_encoder.h"
#include "mlx90393.h"

static int16_t flex_encoder_data[3][MLX_COUNT] = {0};

HAL_StatusTypeDef FlexEncoder_Init(I2C_HandleTypeDef *hi2c) {
    return MLX90393_MUX_Init(hi2c);
}

HAL_StatusTypeDef FlexEncoder_UpdateAll(void) {
    HAL_StatusTypeDef status = HAL_OK;
    int16_t x, y, z;

    for (int i = 0; i < MLX_COUNT; i++) {
        status |= MLX90393_ReadMeasurement(i, &x, &y, &z);
        flex_encoder_data[0][i] = x;
        flex_encoder_data[1][i] = y;
        flex_encoder_data[2][i] = z;
    }

    return status;
}

HAL_StatusTypeDef FlexEncoder_Get(int16_t out[3][MLX_COUNT]) {
    for (int axis = 0; axis < 3; axis++) {
        for (int i = 0; i < MLX_COUNT; i++) {
            out[axis][i] = flex_encoder_data[axis][i];
        }
    }
    return HAL_OK;
}
