/*
 * MT6835.h
 *
 *  Created on: Nov 7, 2024
 *      Author: kyleh
 */

#ifndef MT6835_H
#define MT6835_H

#include "stm32g0xx_hal.h"

// Encoder Registers and Config
#define MT6835_OP_READ       0b0011
#define MT6835_OP_WRITE      0b0110
#define MT6835_OP_PROG       0b1100
#define MT6835_OP_ZERO       0b0101
#define MT6835_OP_ANGLE      0b1010
#define MT6835_CPR           2097152 // Counts per revolution

#define MT6835_REG_ANGLE1    0x003

#define CRC8_POLYNOMIAL 0x07
#define CRC8_INITIAL_VALUE 0x00

#define SPI_1_TIMEOUT 3 //3ms
// We’ll define FULL_RANGE and HALF_RANGE for 21-bit
#define FULL_RANGE (1 << 21)  // 2^21 = 2097152
#define HALF_RANGE (1 << 20)  // 2^20 = 1048576

HAL_StatusTypeDef handle_spi_status(HAL_StatusTypeDef status);

// Function prototypes
void mt6835_init(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
//float mt6835_read_angle(SPI_HandleTypeDef *hspi);
uint32_t mt6835_read_counts(SPI_HandleTypeDef *hspi);


/**
 * @brief Reads current sensor data over SPI, updates:
 *        - singleTurnSensorCounts (21-bit)
 *        - multiTurnSensorCounts  (detecting rollovers)
 *        - Warnings from STATUS bits
 *
 * @param hspi  Pointer to SPI handle
 * @return HAL_StatusTypeDef
 *         - HAL_OK if success
 *         - HAL_TIMEOUT if SPI times out
 *         - HAL_ERROR if SPI error or CRC mismatch
 */
int mt6835_update_counts(SPI_HandleTypeDef *hspi);

/**
 * @brief Get the last single-turn sensor counts (0..2^21-1).
 *
 * @return uint32_t
 */
uint32_t get_counts_single_turn(void);

/**
 * @brief Get the multi-turn rotation count (signed integer).
 *
 * @return int64_t
 */
int64_t get_counts_multi_turn(void);

/**
 * @brief Get the combined (multi-turn << 21) + single-turn as a signed 64-bit.
 *
 * @return int64_t
 */
int64_t get_counts_full(void);

#endif // MT6835_H
