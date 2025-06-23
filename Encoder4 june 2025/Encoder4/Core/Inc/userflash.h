/*
 * userflash.h
 *
 *  Created on: Jan 2, 2025
 *      Author: kyleh
 */

#ifndef INC_USERFLASH_H_
#define INC_USERFLASH_H_
#include "stm32g0xx_hal.h"

/*
 * Define the first page index you want to use for data storage.
 * For example, if your device has 32 KB of Flash (16 pages of 2 KB),
 * and your code uses the first 28 KB (14 pages), you can start at page 14.
 * Adjust this to suit your memory layout.
 */
#define FLASH_DATA_START_PAGE  0x08060000

/* The page size on STM32G0 is typically 2 KB = 2048 bytes. */
#define FLASH_PAGE_SIZE        2048U

/**
  * @brief  Write one 32-bit integer into one whole Flash page, based on index.
  * @param  data:  The 32-bit integer to write.
  * @param  index: Which page "slot" to use (0-based). Each slot = 1 page.
  * @retval HAL_StatusTypeDef (HAL_OK on success, or error status).
  */
HAL_StatusTypeDef UserFlash_WriteInt(uint32_t data, uint32_t index);

/**
  * @brief  Read a 32-bit integer from the Flash page at the given index.
  * @param  index: The same index you used to write.
  * @retval The 32-bit integer stored at that page.
  */
uint32_t UserFlash_ReadInt(uint32_t index);


#endif /* INC_USERFLASH_H_ */
