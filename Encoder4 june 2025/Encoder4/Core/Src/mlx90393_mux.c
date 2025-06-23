///*
// * mlx90393_mux.c
// *
// *  Created on: Mar 18, 2025
// *      Author: thebo
// */
//
//
//
//// mlx90393_mux.c
//#include "mlx90393_mux.h"
//
//extern SPI_HandleTypeDef hspi2;  // From main SPI config
//
//static I2C_HandleTypeDef *pca_i2c = NULL;
//
//// Select one MLX90393 by asserting only its CS line low
//static HAL_StatusTypeDef MLX90393_MUX_Select(int index) {
//    uint8_t output0 = 0xFF;
//    if (index >= 0 && index < MLX_COUNT) {
//        output0 &= ~(1 << (MLX90393_CS_PIN_BASE + index));
//    }
//    return PCA9555_WriteReg(pca_i2c, PCA9555_REG_OUTPUT0, output0);
//}
//
//static void MLX90393_MUX_DeselectAll(void) {
//    PCA9555_WriteReg(pca_i2c, PCA9555_REG_OUTPUT0, 0xFF);  // Set all high (inactive)
//}
//
//HAL_StatusTypeDef MLX90393_MUX_Init(I2C_HandleTypeDef *hi2c) {
//    pca_i2c = hi2c;
//    HAL_StatusTypeDef status = HAL_OK;
//
//    // Init PCA9555: all outputs
//    status |= PCA9555_Init(hi2c);
//
//    // Deselect all MLX90393 sensors
//    MLX90393_MUX_DeselectAll();
//
//    // Initialize each sensor
//    for (int i = 0; i < MLX_COUNT; i++) {
//        MLX90393_MUX_Select(i);
//        HAL_Delay(2);
//        status |= MLX90393_Reset();
//        status |= MLX90393_ConfigureDefault();
//        MLX90393_MUX_DeselectAll();
//        HAL_Delay(2);
//    }
//
//    return status;
//}
//
///**
// * @brief  Reads magnetic (and optional temperature) data from a specific MLX90393 sensor.
// *         The sensor is selected using a multiplexer (PCA9555), and SPI communication is handled accordingly.
// *
// * @param  index Index of the MLX90393 sensor to read (0 to MLX_COUNT - 1).
// * @param  zyxt  Bitfield indicating which axes to read:
// *               - AXIS_X, AXIS_Y, AXIS_Z, AXIS_T (temperature)
// *               - Combine with | (e.g., AXIS_X | AXIS_Y | AXIS_Z)
// * @param  x     Pointer to store X-axis reading (optional, can be NULL)
// * @param  y     Pointer to store Y-axis reading (optional, can be NULL)
// * @param  z     Pointer to store Z-axis reading (optional, can be NULL)
// * @param  t     Pointer to store temperature reading (optional, can be NULL)
// *
// * @retval HAL status (HAL_OK on success, HAL_ERROR if index is out of range or communication fails)
// */
//HAL_StatusTypeDef MLX90393_MUX_Read(int index, int16_t *x, int16_t *y, int16_t *z) {
//    if (index < 0 || index >= MLX_COUNT) return HAL_ERROR;
//
//    HAL_StatusTypeDef status = HAL_OK;
//    MLX90393_MUX_Select(index);
//    HAL_Delay(1);
//    status = MLX90393_ReadMeasurement(x, y, z);
//    HAL_Delay(1);
//
//    MLX90393_MUX_DeselectAll();
//    return status;
//}
//
