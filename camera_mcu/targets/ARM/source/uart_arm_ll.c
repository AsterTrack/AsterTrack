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

#if defined(STM32G0)
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_usart.h"
#include "stm32g0xx_ll_dma.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_system.h"
#endif
#include "compat.h"

#include "uart_driver.h"
#include "util.h"

#include <stdint.h>


const struct UART_DMA_Setup UART[UART_PORT_COUNT] = {
	{ // UART 1
	#if defined(BOARD_OLD)
		.uart = USART2,
	#else
		.uart = USART1,
	#endif
		.DMA = DMA1,
		.DMA_CH_RX = DMA_CHANNEL_2,
		.DMA_CH_TX = DMA_CHANNEL_3,
	#if defined(BOARD_OLD)
		.uartIRQ_RX = USART2_IRQn,
	#else
		.uartIRQ_RX = USART1_IRQn,
	#endif
		.dmaIRQ_RX = DMA1_Channel2_3_IRQn,
		.dmaIRQ_TX = DMA1_Channel2_3_IRQn,
	}
};


/* Functions */

void uart_configure_baudrate(int port, uint32_t baudrate)
{
	struct UART_DMA_Setup u = UART[port];

	bool enabled = LL_USART_IsEnabled(u.uart);
	// Ensure it's disabled
	LL_USART_Disable(u.uart);
	LL_DMA_DisableChannel(u.DMA, u.DMA_CH_RX);
	LL_DMA_DisableChannel(u.DMA, u.DMA_CH_TX);

#if defined(STM32G0)
	LL_USART_SetBaudRate(u.uart, PCLK, LL_USART_PRESCALER_DIV1, LL_USART_OVERSAMPLING_8, baudrate);
#endif

	if (enabled)
	{
		LL_USART_Enable(u.uart);
		LL_DMA_EnableChannel(u.DMA, u.DMA_CH_RX);
	}
}

void uart_driver_init(uint32_t baudrate)
{
	// GPIO Clocks are already active

	// DMA & UART Peripheral clock enable
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
#if defined(STM32G030xx) && defined(BOARD_OLD)
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
	// USART2 on G030/G050 is always PCLK1
#elif defined(STM32G0) && !defined(BOARD_OLD)
	LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
#endif
	int i = 0;
	struct UART_DMA_Setup u = UART[i];

#if defined(STM32G0) && defined(BOARD_OLD)
	// AFIO
	LL_GPIO_SetAFPin_8_15(GPIOA, GPIO_PIN_15, LL_GPIO_AF_1);

	// Init GPIO pins
	// RX: Floating input
	LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_15, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinPull(GPIOA, GPIO_PIN_15, LL_GPIO_PULL_NO);

#elif defined(STM32G0) && !defined(BOARD_OLD)
	// AFIO
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
	LL_SYSCFG_EnablePinRemap(LL_SYSCFG_PIN_RMP_PA11 | LL_SYSCFG_PIN_RMP_PA12);
	LL_GPIO_SetAFPin_8_15(GPIOA, GPIO_PIN_9, LL_GPIO_AF_1);
	LL_GPIO_SetAFPin_8_15(GPIOA, GPIO_PIN_10, LL_GPIO_AF_1);

	// Init GPIO pins
	// TX: Alternate function Push-Pull Output, 50Mhz
	LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinSpeed(GPIOA, GPIO_PIN_9, LL_GPIO_SPEED_FREQ_VERY_HIGH);
	LL_GPIO_SetPinOutputType(GPIOA, GPIO_PIN_9, LL_GPIO_OUTPUT_PUSHPULL);
	// RX: Floating input
	LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinPull(GPIOA, GPIO_PIN_10, LL_GPIO_PULL_NO);
#endif

	// Init UART
	LL_USART_SetTransferDirection(u.uart, LL_USART_DIRECTION_TX_RX);
	LL_USART_ConfigCharacter(u.uart, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_2);
	// For parity:
	//LL_USART_ConfigCharacter(u.uart, LL_USART_DATAWIDTH_9B, LL_USART_PARITY_EVEN, LL_USART_STOPBITS_2);
	LL_USART_SetHWFlowCtrl(u.uart, LL_USART_HWCONTROL_NONE);
	//LL_USART_SetTXRXSwap(u.uart, LL_USART_TXRX_STANDARD);
	//LL_USART_EnableOneBitSamp(u.uart);
	//LL_USART_DisableOneBitSamp(u.uart);
	//LL_USART_SetTXFIFOThreshold(u.uart, LL_USART_FIFOTHRESHOLD_1_4);
	//LL_USART_SetRXFIFOThreshold(u.uart, LL_USART_FIFOTHRESHOLD_1_4);
	//LL_USART_DisableFIFO(u.uart);
	if (baudrate > SYSCLKFRQ*1000000/16)
	{
		LL_USART_SetBaudRate(u.uart, PCLK, LL_USART_PRESCALER_DIV1, LL_USART_OVERSAMPLING_8, baudrate);
		LL_USART_SetOverSampling(u.uart, LL_USART_OVERSAMPLING_8);
	}
	else
	{
		LL_USART_SetBaudRate(u.uart, PCLK, LL_USART_PRESCALER_DIV1, LL_USART_OVERSAMPLING_16, baudrate);
		LL_USART_SetOverSampling(u.uart, LL_USART_OVERSAMPLING_16);		
	}
	LL_USART_ConfigAsyncMode(u.uart);
	// UART RX Interrupt (IDLE)
	LL_USART_EnableIT_IDLE(u.uart);
	// Parity and Receive Error
	LL_USART_EnableIT_PE(u.uart);
	LL_USART_EnableIT_ERROR(u.uart);
	// Enable UART DMA TX & RX
	LL_USART_EnableDMAReq_TX(u.uart);
	LL_USART_EnableDMAReq_RX(u.uart);

	// Setup DMAMUX found on STM32G0 series
#if defined(STM32G0) && defined(BOARD_OLD)
	LL_DMA_SetPeriphRequest(u.DMA, u.DMA_CH_RX, LL_DMAMUX_REQ_USART2_RX);
#elif defined(STM32G0) && !defined(BOARD_OLD)
	//assert(u.uart == USART1);
	LL_DMA_SetPeriphRequest(u.DMA, u.DMA_CH_RX, LL_DMAMUX_REQ_USART1_RX);
	LL_DMA_SetPeriphRequest(u.DMA, u.DMA_CH_TX, LL_DMAMUX_REQ_USART1_TX);
#endif

	// Setup respective DMA Channel for UART RX
	LL_DMA_SetDataTransferDirection(u.DMA, u.DMA_CH_RX, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetChannelPriorityLevel(u.DMA, u.DMA_CH_RX, LL_DMA_PRIORITY_HIGH);
	LL_DMA_SetMode(u.DMA, u.DMA_CH_RX, LL_DMA_MODE_CIRCULAR);
	LL_DMA_SetPeriphIncMode(u.DMA, u.DMA_CH_RX, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(u.DMA, u.DMA_CH_RX, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(u.DMA, u.DMA_CH_RX, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(u.DMA, u.DMA_CH_RX, LL_DMA_MDATAALIGN_BYTE);
	LL_DMA_SetPeriphAddress(u.DMA, u.DMA_CH_RX, (uint32_t)&u.uart->RDR);
	LL_DMA_SetMemoryAddress(u.DMA, u.DMA_CH_RX, (uint32_t)UART_IO[i].rx_buffer);
	LL_DMA_SetDataLength(u.DMA, u.DMA_CH_RX, UART_RX_BUFFER_SIZE);
	// DMA RX Interrupt (HT && TC)
	LL_DMA_EnableIT_HT(u.DMA, u.DMA_CH_RX);
	LL_DMA_EnableIT_TC(u.DMA, u.DMA_CH_RX);
	LL_DMA_EnableIT_TE(u.DMA, u.DMA_CH_RX);

	// Setup respective DMA Channel for UART TX
	LL_DMA_SetDataTransferDirection(u.DMA, u.DMA_CH_TX, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetChannelPriorityLevel(u.DMA, u.DMA_CH_TX, LL_DMA_PRIORITY_HIGH);
	LL_DMA_SetMode(u.DMA, u.DMA_CH_TX, LL_DMA_MODE_NORMAL);
	LL_DMA_SetPeriphIncMode(u.DMA, u.DMA_CH_TX, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(u.DMA, u.DMA_CH_TX, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(u.DMA, u.DMA_CH_TX, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(u.DMA, u.DMA_CH_TX, LL_DMA_MDATAALIGN_BYTE);
	LL_DMA_SetPeriphAddress(u.DMA, u.DMA_CH_TX, (uint32_t)&u.uart->TDR);

	// Enable TX Interrupt (TC)
	LL_DMA_EnableIT_TC(u.DMA, u.DMA_CH_TX);
	// TODO on transfer
	/* LL_DMA_SetMemoryAddress(u.DMA, u.DMA_CH_TX, TX_ADDR);
	LL_DMA_SetDataLength(u.DMA, u.DMA_CH_TX, TX_LEN);
	LL_DMA_EnableChannel(u.DMA, u.DMA_CH_TX); */

	NVIC_SetPriority(u.uartIRQ_RX, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
	NVIC_EnableIRQ(u.uartIRQ_RX);
	NVIC_SetPriority(u.dmaIRQ_RX, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
	NVIC_EnableIRQ(u.dmaIRQ_RX);
	NVIC_SetPriority(u.dmaIRQ_TX, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
	NVIC_EnableIRQ(u.dmaIRQ_TX);

	// Enable UART and DMA RX
	LL_DMA_EnableChannel(u.DMA, u.DMA_CH_RX);
	LL_USART_Enable(u.uart);

	while(!LL_USART_IsActiveFlag_REACK(u.uart)
#if !defined(BOARD_OLD)
		|| !LL_USART_IsActiveFlag_TEACK(u.uart)
#endif
	){}
}

/** Send data over UART port */
inline void uart_send_dma(uint_fast8_t port, const void *data, uint_fast16_t len)
{
	UART_CHARR('/', 'T', 'X', 'D', 'M', 'A');
	LL_DMA_SetDataLength(UART[port].DMA, UART[port].DMA_CH_TX, len);
	LL_DMA_SetMemoryAddress(UART[port].DMA, UART[port].DMA_CH_TX, (uint32_t)data);
	LL_DMA_EnableChannel(UART[port].DMA, UART[port].DMA_CH_TX);
}

static inline bool uart_flush_TX(int port)
{
	LL_DMA_DisableChannel(UART[port].DMA, UART[port].DMA_CH_TX);
	while (LL_DMA_IsEnabledChannel(UART[port].DMA, UART[port].DMA_CH_TX));
	__disable_irq();
	for (int i = 0; i < sizeof(UART_IO[port].tx_queue) / sizeof(UART_IO[port].tx_queue[0]); i++)
	{
		if (UART_IO[port].tx_queue[i].valid)
		{
			uart_send_dma(port, (uint8_t *)UART_IO[port].tx_queue[i].addr, UART_IO[port].tx_queue[i].len);
			UART_IO[port].tx_queue[i].valid = false;
			UART_IO[port].uart_tx = true;
			__enable_irq();
			return true;
		}
	}
	UART_IO[port].uart_tx = false;
	__enable_irq();
	return false;
}

/** USART global interrupt handlers */

void USART1_IRQHandler() __IRQ;
void USART1_IRQHandler()
{ // UART IDLE
	if (LL_USART_IsActiveFlag_IDLE(UART[0].uart))
	{ // RX IDLE
		LL_USART_ClearFlag_IDLE(UART[0].uart);
		uartd_process_port(0, DMA_TAIL(0));
	}
	if (LL_USART_IsActiveFlag_ORE(UART[0].uart))
	{ // Overrun Error
		LL_USART_ClearFlag_ORE(UART[0].uart);
	}
	if (LL_USART_IsActiveFlag_NE(UART[0].uart))
	{ // Noise Error
		LL_USART_ClearFlag_NE(UART[0].uart);
	}
	if (LL_USART_IsActiveFlag_FE(UART[0].uart))
	{ // Frame Error
		LL_USART_ClearFlag_FE(UART[0].uart);
	}
	if (LL_USART_IsActiveFlag_PE(UART[0].uart))
	{ // Parity Error
		LL_USART_ClearFlag_PE(UART[0].uart);
	}
}
void USART2_IRQHandler() __IRQ;
void USART2_IRQHandler()
{ // UART IDLE
	if (LL_USART_IsActiveFlag_IDLE(UART[0].uart))
	{ // RX IDLE
		LL_USART_ClearFlag_IDLE(UART[0].uart);
		uartd_process_port(0, DMA_TAIL(0));
	}
}
void DMA1_Channel2_3_IRQHandler() __IRQ;
void DMA1_Channel2_3_IRQHandler()
{
	if (LL_DMA_IsActiveFlag_TE2(DMA1))
	{ // RX FAIL
		LL_DMA_ClearFlag_TE2(DMA1);
	}
	if (LL_DMA_IsActiveFlag_TC2(DMA1))
	{ // RX Full
		LL_DMA_ClearFlag_TC2(DMA1);
		uartd_process_port(0, DMA_TAIL(0));
	}
	if (LL_DMA_IsActiveFlag_HT2(DMA1))
	{ // RX Half
		LL_DMA_ClearFlag_HT2(DMA1);
		uartd_process_port(0, DMA_TAIL(0));
	}
	if (LL_DMA_IsActiveFlag_TE3(DMA1))
	{ // TX FAIL
		LL_DMA_ClearFlag_TE3(DMA1);
	}
	if (LL_DMA_IsActiveFlag_TC3(DMA1))
	{ // TX Complete
		LL_DMA_ClearFlag_TC3(DMA1);
		uart_flush_TX(0);
	}
}