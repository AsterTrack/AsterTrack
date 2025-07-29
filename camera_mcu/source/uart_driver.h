/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef __UART_DRIVER_H
#define __UART_DRIVER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

#include "uartd_conf.h"


/* Structures */

struct UART_DMA_Setup {
	USART_TypeDef *uart;
	DMA_TypeDef *DMA;
	uint32_t DMA_CH_RX;
	uint32_t DMA_CH_TX;
	IRQn_Type uartIRQ_RX;
	IRQn_Type dmaIRQ_RX;
	IRQn_Type dmaIRQ_TX;
};
const static int SZ_TX_QUEUE = 5;
struct UART_IO_State {
	struct {
		bool valid;
		uint32_t addr;
		uint32_t len;
	} tx_queue[5];
	int tx_queue_pos;
	bool uart_tx;
	uint8_t rx_alloc[UART_HEADROOM+UART_RX_BUFFER_SIZE];
	uint8_t *rx_buffer;
};

extern const struct UART_DMA_Setup UART[UART_PORT_COUNT];
extern struct UART_IO_State UART_IO[UART_PORT_COUNT];

#define DMA_TAIL(port) (UART_RX_BUFFER_SIZE - DMA_CH(UART[port].DMA, UART[port].DMA_CH_RX)->COUNTER)

// Implemented generally
void uartd_process_port(uint_fast8_t port, uint_fast16_t tail);

// Implemented specfically
void uart_driver_init();
void uart_send_dma(uint_fast8_t port, const uint8_t* data, uint_fast16_t len);
void uart_configure_baudrate(int port, uint32_t baudrate);

#ifdef __cplusplus
}
#endif

#endif // __UART_DRIVER_H