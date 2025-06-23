/*
 * mlx90393.c
 *
 *  Created on: Mar 18, 2025
 *      Author: thebo
 */

#include "mlx90393.h"
#include "pca9555.h"

extern SPI_HandleTypeDef hspi2;
static I2C_HandleTypeDef *pca_i2c = NULL;

static HAL_StatusTypeDef MLX90393_MUX_Select(int index) {
	uint8_t output0 = 0xFF;
	if (index >= 0 && index < MLX_COUNT) {
		output0 &= ~(1 << (MLX90393_CS_PIN_BASE + index));
	}
	return PCA9555_WriteReg(pca_i2c, PCA9555_REG_OUTPUT0, output0);
}

static void MLX90393_MUX_DeselectAll(void) {
	PCA9555_WriteReg(pca_i2c, PCA9555_REG_OUTPUT0, 0xFF);
}

HAL_StatusTypeDef MLX90393_Reset(int index) {
	if (index < 0 || index >= MLX_COUNT)
		return HAL_ERROR;

	HAL_StatusTypeDef status = HAL_OK;
	uint8_t cmd[1], resp[1];

	MLX90393_MUX_Select(index);
	HAL_Delay(1);

	cmd[0] = MLX_CMD_EX;
	status |= HAL_SPI_TransmitReceive(&hspi2, cmd, resp, 1, 10);
	HAL_Delay(1);

	cmd[0] = MLX_CMD_RT;
	status |= HAL_SPI_TransmitReceive(&hspi2, cmd, resp, 1, 10);
	HAL_Delay(2);

	MLX90393_MUX_DeselectAll();
	return status;
}

HAL_StatusTypeDef MLX90393_ConfigureDefault(int index) {
	if (index < 0 || index >= MLX_COUNT)
		return HAL_ERROR;

	HAL_StatusTypeDef status = HAL_OK;
	uint8_t cmd[5], resp[5];

	// Register 0x00 = 0x00: GAIN_SEL=0, HALLCONF=0
	cmd[0] = MLX_CMD_WR;
	cmd[1] = 0x00;
	cmd[2] = (0x00 << 2);
	cmd[3] = 0x00;
	cmd[4] = 0x00;

	MLX90393_MUX_Select(index);
	HAL_Delay(1);
	status |= HAL_SPI_TransmitReceive(&hspi2, cmd, resp, 5, 10);
	MLX90393_MUX_DeselectAll();

	HAL_Delay(1);

	// Register 0x02 = 0x30: RES_XYZ=3, OSR=0, DIG_FILT=0
	cmd[2] = (0x02 << 2);
	cmd[3] = 0x00;
	cmd[4] = 0x30;

	MLX90393_MUX_Select(index);
	HAL_Delay(1);
	status |= HAL_SPI_TransmitReceive(&hspi2, cmd, resp, 5, 10);
	MLX90393_MUX_DeselectAll();

	HAL_Delay(1);

	// Send HS (Store to EEPROM)
	cmd[0] = MLX_CMD_HS;

	MLX90393_MUX_Select(index);
	HAL_Delay(1);
	status |= HAL_SPI_TransmitReceive(&hspi2, cmd, resp, 1, 10);
	HAL_Delay(20);  // EEPROM write delay
	MLX90393_MUX_DeselectAll();

	return status;
}

HAL_StatusTypeDef MLX90393_ReadMeasurement(int index, int16_t *x, int16_t *y,
		int16_t *z) {
	if (index < 0 || index >= MLX_COUNT)
		return HAL_ERROR;

	HAL_StatusTypeDef status;
	uint8_t tx[6] = { 0 };
	uint8_t rx[20] = { 0 };

	// --- Start Single Measurement ---
	tx[0] = MLX_CMD_SM | AXIS_X | AXIS_Y | AXIS_Z;
	//tx[0] = 0x16;
	tx[1] = 0x00;

	MLX90393_MUX_Select(index);
	//HAL_Delay(1);
	status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, 10);
	MLX90393_MUX_DeselectAll();
	if (status != HAL_OK) {
		return status;
	}

	HAL_Delay(50);  // wait for conversion

	// --- Read Measurement ---
	//tx[0] = MLX_CMD_RM | AXIS_X | AXIS_Y | AXIS_Z;
	for (int i = 1; i < 7; i++)
		tx[i] = 0x00;

	MLX90393_MUX_Select(index);
	//HAL_Delay(1);
	status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 8, 10);
	MLX90393_MUX_DeselectAll();
	if (status != HAL_OK) {
		return status;
	}

//	if ((rx[0] & 0xF0) != 0x20)
//		return HAL_ERROR; // SM status check

	*z = (rx[2] << 8) | rx[3];
	*y = (rx[4] << 8) | rx[5];
	*x = (rx[6] << 8) | rx[7];

//	*z = (rx[1] << 8) | rx[2];
//	*y = (rx[3] << 8) | rx[4];
//	*x = (rx[5] << 8) | rx[6];

	return HAL_OK;
}

HAL_StatusTypeDef MLX90393_MUX_Init(I2C_HandleTypeDef *hi2c) {
	pca_i2c = hi2c;
	HAL_StatusTypeDef status = HAL_OK;

	status |= PCA9555_Init(hi2c);
	MLX90393_MUX_DeselectAll();

	for (int i = 0; i < MLX_COUNT; i++) {
		status |= MLX90393_Reset(i);
		status |= MLX90393_ConfigureDefault(i);
	}

	return status;
}
