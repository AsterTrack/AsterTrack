/*! ----------------------------------------------------------------------------
 *  @file    stm32_i2c_bootloader.c
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

#include "stm32_bootloader.hpp"

#include "stm32_bootloader_platform.hpp"

#define ACK 0x79
#define NACK 0x1F
#define BUSY 0x76

#define SLAVE_SERIAL_TIMEOUT 15000
#define SLAVE_SERIAL_TIMEOUT_ERASE 120000

pRESULT sendBytesWithAck(const uint8_t *bytes, int count, int len,
												 int timeoutInMs);

/**
 * @brief  address point to the next 0-256 byte block
 * @param  loadAddr     flash address to shift
 * @retval will return RES_FAIL if len>256
 */
pRESULT incrementLoadAddress(uint8_t *loadAddr, uint16_t len);

pRESULT loadAddress(const uint8_t *address);

pRESULT bootloaderCheckAndErase(void) {
	/*
									Define How much flash you want to erase
									For STM32L432, flash size =  eraseLoopNum*sectorsPerLoop*2k
	*/
	uint16_t eraseLoopNum = 10; // up to 30
	uint16_t sectorsPerLoop = 4;

	platform_delay_ms(1000);
#if (BOOTLOADER_PORT == BOOTLOADER_UART)
	if (bootloaderSync() != RES_OK) { // only available in UART protocol
		return RES_FAIL;
	}
	platform_delay_ms(2000);
#endif
	if (bootloaderGet() != RES_OK) {
		return RES_FAIL;
	}
	if (bootloaderVersion() != RES_OK) {
		return RES_FAIL;
	}
	if (bootloaderId() != RES_OK) {
		return RES_FAIL;
	}
	/*Uncomment this command if flash is protected */
	//		if(bootloaderReleaseMemProtect()!=RES_OK){
	//			return RES_FAIL;
	//		}
	/*Complete erase has not been verified */
	//		if(bootloaderExtErase()!=RES_OK){
	//			return RES_FAIL;
	//		}
	platform_delay_ms(1000);
	for (int i = 0; i < eraseLoopNum; i++) {
		if (bootloaderErasePages(i * sectorsPerLoop + 0, sectorsPerLoop) !=
				RES_OK) {
			return RES_FAIL;
		}
		LogDebugInfo(("Slave MCU IAP: erased 4 page"));
	}
	return RES_OK;
}
#if (BOOTLOADER_PORT == BOOTLOADER_UART)
pRESULT bootloaderSync(void) {
	LogDebugInfo(("Slave MCU IAP: start slave MCU IAP"));
	uint8_t cmd[] = {0x7F};
	return sendBytesWithAck(cmd, sizeof(cmd), 1, 1000);
}
#else
pRESULT bootloaderSync(void) {
	return RES_FAIL;
}
#endif

pRESULT bootloaderGet(void) {
	LogDebugInfo(("Slave MCU IAP: get device info"));
	uint8_t cmd[] = {0x00, 0xFF};

	pRESULT res = sendBytesWithAck(cmd, sizeof(cmd), 1, 1000);
	if (res != RES_OK) {
		LogDebugInfo(("Slave MCU IAP: get device info failed"));
		return res;
	}
	uint8_t tmpData[13];
	res = platform_read(tmpData, 13);
	LogDebugInfo(("Slave MCU IAP: bootloader version:"));
	LogDebugInfoHEX(tmpData[1]);
	res = platform_read(tmpData, 1);
	return res;
}

pRESULT bootloaderVersion(void) {
	LogDebugInfo(("Slave MCU IAP: GET VERSION "));
	uint8_t cmd[] = {0x01, 0xFE};
	pRESULT res = sendBytesWithAck(cmd, sizeof(cmd), 1, 1000);
	if (res != RES_OK) {
		return res;
	}

#if (BOOTLOADER_PORT == BOOTLOADER_I2C)
	uint8_t tmpData[1];
	res = platform_read(tmpData, 1); // IIC protocol read 1
																											// byte
#elif (BOOTLOADER_PORT == BOOTLOADER_UART)
	uint8_t tmpData[3];
	res = platform_read(tmpData, 3); // UART protocol read 3 bytes
#endif
	LogDebugInfo(("Slave MCU IAP: bootloader version:"));
	LogDebugInfoHEX(tmpData[0]);
	res = platform_read(tmpData, 1);
	return res;
}

pRESULT bootloaderId(void) {
	LogDebugInfo(("Slave MCU IAP: CHECK ID"));
	uint8_t cmd[] = {0x02, 0xFD};
	pRESULT res = sendBytesWithAck(cmd, sizeof(cmd), 1, 1000);
	if (res != RES_OK) {
		return res;
	}
	uint8_t tmpData[3];
	res = platform_read(tmpData, 3);
	res = platform_read(tmpData, 1);
	LogDebugInfo(("Slave MCU IAP: Device type ID:"));
	LogDebugInfoHEX(tmpData[1]);
	LogDebugInfoHEX(tmpData[2]);
	return res;
}

pRESULT bootloaderErasePages(uint16_t startPageIdx, uint16_t pageNum) {
	if (pageNum == 0) {
		return RES_OK;
	}
	LogDebugInfo(("Slave MCU IAP: ERASE MEMORY"));
	uint8_t cmd[] = {0x44, 0xBB};
	if (sendBytesWithAck(cmd, sizeof(cmd), 1, 1000) == RES_OK) {
		uint8_t tmpData[pageNum * 2 + 4];
		uint16_t tmpN = pageNum - 1; // need to send N-1
		tmpData[0] = (uint8_t)(tmpN >> 8);
		tmpData[1] = (uint8_t)(tmpN & 0xFF);
#if (BOOTLOADER_PORT == BOOTLOADER_UART)
		platform_write(tmpData, 2);
#else
		tmpData[2] = tmpData[0] ^ tmpData[1];
		if (sendBytesWithAck(tmpData, 3, 1, 1000) != RES_OK) {
			LogDebugInfo(("Slave MCU IAP: Receive NACK while sending page num"));
			return RES_FAIL;
		}
#endif
#if (BOOTLOADER_PORT == BOOTLOADER_UART)
		tmpData[pageNum * 2] =
				tmpData[0] ^ tmpData[1]; // UART port just send 1 sum check
#else
		tmpData[pageNum * 2] = 0;
#endif
		for (uint8_t i = 0; i < pageNum; i++) {
			tmpN = startPageIdx + i;
			tmpData[i * 2] = (uint8_t)(tmpN >> 8);
			tmpData[i * 2 + 1] = (uint8_t)(tmpN & 0xFF);
			tmpData[pageNum * 2] = tmpData[pageNum * 2] ^ tmpData[i * 2];
			tmpData[pageNum * 2] = tmpData[pageNum * 2] ^ tmpData[i * 2 + 1];
		}
		pRESULT res = platform_write(tmpData, pageNum * 2 + 1);
		if (res != RES_OK) {
			LogDebugInfo(("Slave MCU IAP: failed to send"));
			return RES_FAIL;
		}
		uint8_t repeater = 0;
		while (repeater < 20) {
			res = platform_read(tmpData, 1);
			if (res == RES_OK) {
				if (tmpData[0] == ACK) {
					LogDebugInfo(("Slave MCU IAP: Erase success"));
					return RES_OK;
				} else {
					LogDebugInfo(
							("Slave MCU IAP: Memory erased although 1F is returned"));
					LogDebugInfoHEX(tmpData[0]);
					return RES_OK; // Memory erased although 1F is returned"
				}
			}
			repeater++;
		}
	}
	LogDebugInfo(("Slave MCU IAP: erase failed"));
	return RES_FAIL;
}

pRESULT bootloaderExtErase(void) {
	LogDebugInfo(("Slave MCU IAP: EXTENDED ERASE MEMORY"));
	uint8_t cmd[] = {0x44, 0xBB};

	if (sendBytesWithAck(cmd, sizeof(cmd), 1, 1000) == RES_OK) {
		uint8_t tmpData[] = {0xFF, 0xFF, 0x00};

		if (sendBytesWithAck(tmpData, sizeof(tmpData), 1,
												 SLAVE_SERIAL_TIMEOUT_ERASE) == RES_OK) {
			return RES_OK;
		}
	}
	LogDebugInfo(("Slave MCU IAP: Timeout, erase failed"));
	return RES_FAIL;
}
pRESULT bootloaderReleaseMemProtect(void) {
	LogDebugInfo(("Slave MCU IAP: UPROTECT MEMORY"));
#ifdef USE_NO_STRETCH_COMMANDS
	uint8_t cmd[] = {0x74, 0x8B};
#else
	uint8_t cmd[] = {0x73, 0x8C};
#endif
	if (sendBytesWithAck(cmd, sizeof(cmd), 1, 1000) == RES_FAIL)
		return RES_FAIL;

	uint8_t tmpData[1];
	while (true)
	{
		pRESULT res = platform_read(tmpData, 1);
		if (res == RES_OK) {
			if (tmpData[0] == ACK) {
				LogDebugInfo(("Slave MCU IAP: Memory Unprotect Success"));
				return RES_OK;
			} else if (tmpData[0] == BUSY) {
				LogDebugInfo(("Slave MCU IAP: Busy unprotecting..."));
				platform_delay_ms(1);
			} else {
				LogDebugInfo(("Slave MCU IAP: Memory Unprotect Failure, code:"));
				LogDebugInfoHEX(tmpData[0]);
				return RES_FAIL;
			}
		} else {
			LogDebugInfo(("Slave MCU IAP: Read timeout or insefficient length"));
			return RES_FAIL;
		}
	}
}

pRESULT bootloaderWrite(void) {
	LogDebugInfo(("Slave MCU IAP: WRITE MEMORY"));
#ifdef USE_NO_STRETCH_COMMANDS
	uint8_t cmd[2] = {0x32, 0xCD};
#else
	uint8_t cmd[2] = {0x31, 0xCE};
#endif
	return sendBytesWithAck(cmd, sizeof(cmd), 1, 1000);
}

pRESULT bootloaderRead(void) {
	LogDebugInfo(("Slave MCU IAP: READ MEMORY"));
	uint8_t cmd[2] = {0x11, 0xEE};

	return sendBytesWithAck(cmd, sizeof(cmd), 1, 1000);
}

pRESULT loadAddress(const uint8_t *address) {
	uint8_t tmpData[5];
	memcpy(tmpData, address, 4);
	tmpData[4] = address[0] ^ address[1] ^ address[2] ^ address[3];

	return sendBytesWithAck(tmpData, sizeof(tmpData), 1, 1000);
}

pRESULT sendBytesWithAck(const uint8_t *bytes, int count, int len,
												 int timeoutInMs) {
	pRESULT res = platform_write(bytes, count);
	if (res != RES_OK) {
		LogDebugInfo(("Slave MCU IAP: failed to send"));
		return RES_FAIL;
	}
	uint8_t dataBuf[len];
	res = platform_read(dataBuf, len);
	if (res == RES_OK) {
		if (dataBuf[len - 1] == ACK) {
			LogDebugInfo(("Slave MCU IAP: Receive ACK"));
			return RES_OK;
		} else {
			LogDebugInfo(("Slave MCU IAP: Receive NACK"));
			LogDebugInfoHEX(dataBuf[0]);
			return RES_FAIL;
		}
	} else {
		LogDebugInfo(("Slave MCU IAP: failed to receive ACK"));
		return RES_FAIL;
	}
}

pRESULT incrementAddress(uint8_t *loadAddr, uint16_t len) {
	if (len > 256) {
		return RES_FAIL;
	}
	if (len + loadAddr[3] >= 256) {

		loadAddr[2] += 0x1;

		if (loadAddr[2] == 0) {
			loadAddr[1] += 0x1;

			if (loadAddr[1] == 0) {
				loadAddr[0] += 0x1;
			}
		}
	}
	loadAddr[3] = 0xFF & (len + loadAddr[3]);
	return RES_OK;
}

pRESULT flashPage(const uint8_t *address, const uint8_t *dataBuf, uint16_t len) {
	if (len > 256) {
		return RES_FAIL;
	}
	if (len == 0) {
		return RES_OK;
	}
	uint8_t writeNum = 0xFF & (len - 1);
	uint8_t tx_data[writeNum + 3];
	tx_data[0] = writeNum;
	tx_data[writeNum + 2] = 0x00 ^ tx_data[0];
	for (int i = 0; i <= writeNum; i++) {
		tx_data[1 + i] = dataBuf[i];
		tx_data[writeNum + 2] ^= dataBuf[i];
	}

	if (bootloaderWrite() == RES_FAIL) return RES_FAIL;
	if (loadAddress(address) == RES_FAIL) return RES_FAIL;

	platform_write(tx_data, writeNum + 3);

	uint8_t tmpData[1];
	while (true)
	{
		pRESULT res = platform_read(tmpData, 1);
		if (res == RES_OK) {
			if (tmpData[0] == ACK) {
				LogDebugInfo(("Slave MCU IAP: Flash Success"));
				return RES_OK;
			} else if (tmpData[0] == BUSY) {
				LogDebugInfo(("Slave MCU IAP: Busy writing..."));
				platform_delay_ms(1);
			} else {
				LogDebugInfo(("Slave MCU IAP: Flash Failure, code:"));
				LogDebugInfoHEX(tmpData[0]);
				return RES_FAIL;
			}
		} else {
			LogDebugInfo(("Slave MCU IAP: Read timeout or insefficient length"));
			return RES_FAIL;
		}
	}
}

pRESULT verifyPage(const uint8_t *address, const uint8_t *dataBuf, uint16_t len) {
	LogDebugInfo(("Slave MCU IAP: Reading page"));
	uint8_t param[] = {0xFF, 0x00};
	uint8_t readNum = 0xFF & (len - 1);
	uint8_t tx_data[2];
	tx_data[0] = readNum;
	tx_data[1] = 0xFF ^ readNum;
	bootloaderRead();
	loadAddress(address);
	pRESULT res = sendBytesWithAck(tx_data, 2, 1, 1000);
	if (res != RES_OK) {
		LogDebugInfo(("Slave MCU IAP: sending read len failed"));
		return RES_FAIL;
	}
	uint8_t rx_data[len + 1];
	res = platform_read(rx_data, len);
	if (res == RES_OK) {

		LogDebugInfo(("Slave MCU IAP: Read Success"));
		if (memcmp(dataBuf, rx_data, len) == 0) {
			return RES_OK;
		} else {
			LogDebugInfo(("Slave MCU IAP: data not match"));
//			for(int i =0;i<len;i++){
//				LogDebugInfoHEX(dataBuf[i] );
//				LogDebugInfoHEX(rx_data[i] );
//			}
			return RES_FAIL;
		}

	} else {
		LogDebugInfo(("Slave MCU IAP: Read timeout or insefficient length"));
		return RES_FAIL;
	}

	return RES_FAIL;
}
