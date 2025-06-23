/*
 * userflash.c
 *
 *  Created on: Jan 2, 2025
 *      Author: kyleh
 */

#include "userflash.h"

/*
 * Helper function to compute the start address of the page for a given index.
 * Each page is 2 KB. If you choose page 14 as your start,
 * then index=0 -> page 14, index=1 -> page 15, etc.
 */
static uint32_t UserFlash_GetPageAddress(uint32_t index)
{
    /* Page 0 (of the entire Flash) starts at 0x0800 0000 on STM32G0.
     * So pageAddress = FLASH_BASE + (pageNumber * FLASH_PAGE_SIZE).
     */
    uint32_t pageNumber   = FLASH_DATA_START_PAGE + index;
    uint32_t pageAddress  = FLASH_BASE + (pageNumber * FLASH_PAGE_SIZE);
    return pageAddress;
}

/**
  * @brief  Write a single 32-bit integer to a dedicated Flash page.
  *         1. Unlock
  *         2. Erase the page
  *         3. Write the integer (as 64 bits)
  *         4. Lock
  */
HAL_StatusTypeDef UserFlash_WriteInt(uint32_t data, uint32_t index)
{
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t pageError = 0U;

    /* 1) Unlock the Flash for write/erase access */
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
    {
        return status;
    }

    /* 2) Erase one entire page corresponding to 'index' */
    {
        /* The pageNumber is FLASH_DATA_START_PAGE + index */
        uint32_t pageNumber = FLASH_DATA_START_PAGE + index;

        eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
        eraseInit.Page      = pageNumber;           // which page to erase
        eraseInit.NbPages   = 1;                    // erase exactly one page
        eraseInit.Banks     = FLASH_BANK_1;         // STM32G0 typically uses BANK_1 only

        status = HAL_FLASHEx_Erase(&eraseInit, &pageError);
        if (status != HAL_OK)
        {
            /* Lock the Flash before returning */
            (void)HAL_FLASH_Lock();
            return status;
        }
    }

    /* 3) Write the 32-bit data (packed into 64 bits) at the start of that page */
    {
        union
        {
            uint64_t doubleWord;  // 64-bit
            uint32_t word[2];     // two 32-bit halves
        } dataToWrite;

        dataToWrite.word[0] = data;
        dataToWrite.word[1] = 0xFFFFFFFF;  // filler for the upper 32 bits

        uint32_t pageAddress = UserFlash_GetPageAddress(index);

        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   pageAddress,
                                   dataToWrite.doubleWord);
        if (status != HAL_OK)
        {
            /* Lock the Flash before returning */
            (void)HAL_FLASH_Lock();
            return status;
        }
    }

    /* 4) Lock the Flash after writing */
    status = HAL_FLASH_Lock();
    return status;
}

/**
  * @brief  Read a single 32-bit integer from the page corresponding to 'index'.
  * @param  index: 0-based index, as passed to WriteInt.
  * @retval The 32-bit integer stored at that page.
  */
uint32_t UserFlash_ReadInt(uint32_t index)
{
    /* Reading from Flash is just a direct pointer dereference. */
    uint32_t pageAddress = UserFlash_GetPageAddress(index);
    return *(volatile uint32_t *)pageAddress;
}
