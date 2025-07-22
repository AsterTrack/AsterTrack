/*! ----------------------------------------------------------------------------
 *  @file    stm32_i2c_bootloader.h
 *  @brief
 *
 * @attention
 *
 * Copyright
 *
 * All rights reserved.
 *
 * @author Kai Zhao
 *
 * Modified by Seneral to remove control over entering/leaving bootloader to make it behave more like a library
 */
#ifndef _STM32_I2C_BOOTLOADER_H_
#define _STM32_I2C_BOOTLOADER_H_

#include <cstdint>

typedef enum pRESULT { RES_OK, RES_FAIL } pRESULT;

/**
 * @brief  flash data to STM32
 * @param  address     flash address
 * @param  data        data to write
 * @param  len         data length, no greater than 256
 * @retval result 0 = RES_OK  1 = RES_FAIL
 */
pRESULT flashPage(const uint8_t *address, const uint8_t *data, uint16_t len);

/**
 * @brief  compare the data to the bytes read from STM32
 * @param  address     flash address
 * @param  data        data to compare
 * @retval result 0 = RES_OK  1 = RES_FAIL
 */
pRESULT verifyPage(const uint8_t *address, const uint8_t *data, uint16_t len);

/**
 * @brief  increment the given address by the given length in bytes
 * @param  address     address
 * @param  len         page length, no greater than 256
 * @retval result 0 = RES_OK  1 = RES_FAIL
 */
pRESULT incrementAddress(uint8_t *address, uint16_t len);

/**
 * @brief  read and print some useful information,erase sectors. You may adjust
 * the sectors num based your firmware size
 * @retval RES_OK or RES_FAIL
 */
pRESULT bootloaderCheckAndErase(void);

pRESULT bootloaderSync(void);
pRESULT bootloaderGet(void);
pRESULT bootloaderVersion(void);
pRESULT bootloaderId(void);
pRESULT bootloaderWrite(void);
pRESULT bootloaderRead(void);
pRESULT bootloaderReleaseMemProtect(void);
pRESULT bootloaderExtErase(void); // not verified
/**
 * @brief Erase continuous sectors, in STM32L432, sectors are all in 2K,
 * @param  startPageIdx          start sector/page index
 * @param  pageNum               page/sector num in total (recomend to erase 1-4
 * sectors at a time, have not verified more sectors)
 * @retval RES_OK or RES_FAIL
 */
pRESULT bootloaderErasePages(uint16_t startPageIdx, uint16_t pageNum);

#endif //_STM32_I2C_BOOTLOADER_H_
