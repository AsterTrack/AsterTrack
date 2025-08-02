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

#include "ch32v30x_rcc.h"
#include "ch32v30x_gpio.h"
#include "ch32v30x_usart.h"
#include "ch32v30x_dma.h"
#include "compat.h"

#include "uart_driver.h"
#include "util.h"

#include <stdint.h>


const struct UART_DMA_Setup UART[UART_PORT_COUNT] = {
	{ // UART 1
		.uart = USART1,
		.GPIOxTX = GPIOA,
		.GPIOxRX = GPIOA,
		.PinTX = GPIO_PIN_9,
		.PinRX = GPIO_PIN_10,
		.DMA = DMA1,
		.DMA_CH_RX = DMA_CHANNEL_5,
		.DMA_CH_TX = DMA_CHANNEL_4,
		.uartIRQ_RX = USART1_IRQn,
		.dmaIRQ_RX = DMA1_Channel5_IRQn,
		.dmaIRQ_TX = DMA1_Channel4_IRQn,
		.peripheral = PCLK2, // Only UART1 runs off of PCLK2
	},
	{ // UART 2
		.uart = USART2,
		.GPIOxTX = GPIOA,
		.GPIOxRX = GPIOA,
		.PinTX = GPIO_PIN_2,
		.PinRX = GPIO_PIN_3,
		.DMA = DMA1,
		.DMA_CH_RX = DMA_CHANNEL_6,
		.DMA_CH_TX = DMA_CHANNEL_7,
		.uartIRQ_RX = USART2_IRQn,
		.dmaIRQ_RX = DMA1_Channel6_IRQn,
		.dmaIRQ_TX = DMA1_Channel7_IRQn,
		.peripheral = PCLK1,
	},
	{ // UART 3
		.uart = USART3,
		.GPIOxTX = GPIOB,
		.GPIOxRX = GPIOB,
		.PinTX = GPIO_PIN_10,
		.PinRX = GPIO_PIN_11,
		.DMA = DMA1,
		.DMA_CH_RX = DMA_CHANNEL_3,
		.DMA_CH_TX = DMA_CHANNEL_2,
		.uartIRQ_RX = USART3_IRQn,
		.dmaIRQ_RX = DMA1_Channel3_IRQn,
		.dmaIRQ_TX = DMA1_Channel2_IRQn,
		.peripheral = PCLK1,
	},
	{ // UART 4
		.uart = UART4,
		.GPIOxTX = GPIOC,
		.GPIOxRX = GPIOC,
		.PinTX = GPIO_PIN_10,
		.PinRX = GPIO_PIN_11,
		.DMA = DMA2,
		.DMA_CH_RX = DMA_CHANNEL_3,
		.DMA_CH_TX = DMA_CHANNEL_5,
		.uartIRQ_RX = UART4_IRQn,
		.dmaIRQ_RX = DMA2_Channel3_IRQn,
		.dmaIRQ_TX = DMA2_Channel5_IRQn,
		.peripheral = PCLK1,
	},
	{ // UART 5
		.uart = UART5,
		.GPIOxTX = GPIOC,
		.GPIOxRX = GPIOD,
		.PinTX = GPIO_PIN_12,
		.PinRX = GPIO_PIN_2,
		.DMA = DMA2,
		.DMA_CH_RX = DMA_CHANNEL_2,
		.DMA_CH_TX = DMA_CHANNEL_4,
		.uartIRQ_RX = UART5_IRQn,
		.dmaIRQ_RX = DMA2_Channel2_IRQn,
		.dmaIRQ_TX = DMA2_Channel4_IRQn,
		.peripheral = PCLK1,
	},
	{ // UART 6
		.uart = UART6,
		.GPIOxTX = GPIOC,
		.GPIOxRX = GPIOC,
		.PinTX = GPIO_PIN_0,
		.PinRX = GPIO_PIN_1,
		.DMA = DMA2,
		.DMA_CH_RX = DMA_CHANNEL_7,
		.DMA_CH_TX = DMA_CHANNEL_6,
		.uartIRQ_RX = UART6_IRQn,
		.dmaIRQ_RX = DMA2_Channel7_IRQn,
		.dmaIRQ_TX = DMA2_Channel6_IRQn,
		.peripheral = PCLK1,
	},
	{ // UART 7
		.uart = UART7,
		.GPIOxTX = GPIOC,
		.GPIOxRX = GPIOC,
		.PinTX = GPIO_PIN_2,
		.PinRX = GPIO_PIN_3,
		.DMA = DMA2,
		.DMA_CH_RX = DMA_CHANNEL_9,
		.DMA_CH_TX = DMA_CHANNEL_8,
		.uartIRQ_RX = UART7_IRQn,
		.dmaIRQ_RX = DMA2_Channel9_IRQn,
		.dmaIRQ_TX = DMA2_Channel8_IRQn,
		.peripheral = PCLK1,
	},
	{ // UART 8
		.uart = UART8,
		.GPIOxTX = GPIOC,
		.GPIOxRX = GPIOC,
		.PinTX = GPIO_PIN_4,
		.PinRX = GPIO_PIN_5,
		.DMA = DMA2,
		.DMA_CH_RX = DMA_CHANNEL_11,
		.DMA_CH_TX = DMA_CHANNEL_10,
		.uartIRQ_RX = UART8_IRQn,
		.dmaIRQ_RX = DMA2_Channel11_IRQn,
		.dmaIRQ_TX = DMA2_Channel10_IRQn,
		.peripheral = PCLK1,
	}
};


/* Functions */

void uart_configure_baudrate(int port, uint32_t baudrate)
{
	struct UART_DMA_Setup u = UART[port];

	bool enabled = u.uart->CTLR1 & USART_CTLR1_UE;
	// Ensure it's disabled
	u.uart->CTLR1 &= ~USART_CTLR1_UE;
	DMA_CH(u.DMA, u.DMA_CH_RX)->CONTROL &= ~DMA_CFGR1_EN;
	DMA_CH(u.DMA, u.DMA_CH_TX)->CONTROL &= ~DMA_CFGR1_EN;
	// Read stat & data register to clear flags
	uint8_t stat = u.uart->STATR;
	uint32_t data = u.uart->DATAR;
	(void) stat;
	(void) data;

	// Set Baud Rate
	uint32_t intDiv = (25 * u.peripheral) / (4 * baudrate);
	uint32_t tmp = (intDiv / 100) & 0xFFF;
	uint32_t fracDiv = intDiv - (100 * tmp);
	u.uart->BRR = (tmp << 4) | (((fracDiv * 16 + 50) / 100) & 0xF);

	if (enabled)
	{
		u.uart->CTLR1 |= USART_CTLR1_UE;
		DMA_CH(u.DMA, u.DMA_CH_RX)->CONTROL |=	DMA_CFGR1_EN;
	}
}

void uart_driver_init()
{
	// GPIO Clocks are already active

	// DMA & UART Peripheral clock enable
	RCC->AHBPCENR |= RCC_AHBPeriph_DMA1 | RCC_AHBPeriph_DMA2;
	RCC->APB2PCENR |= RCC_APB2Periph_USART1;
	RCC->APB1PCENR |= RCC_APB1Periph_USART2 | RCC_APB1Periph_USART3 | RCC_APB1Periph_UART4 | RCC_APB1Periph_UART5 | RCC_APB1Periph_UART6 | RCC_APB1Periph_UART7 | RCC_APB1Periph_UART8;

	for (uint_fast8_t i = 0; i < sizeof(UART)/sizeof(struct UART_DMA_Setup); i++)
	{
		struct UART_DMA_Setup u = UART[i];

		// Init GPIO pins
		// TX: Alternate function Push-Pull Output, 50Mhz
		GPIO_CFG(u.GPIOxTX, u.PinTX, GPIO_AF_PP_OUT | GPIO_Speed_50MHz);
		// RX: Floating input
		GPIO_CFG(u.GPIOxRX, u.PinRX, GPIO_FLOATING_IN);

		// Init UART
		u.uart->CTLR1 = USART_WordLength_8b | USART_Parity_No | USART_Mode_Tx | USART_Mode_Rx;
		// For parity:
		//u.uart->CTLR1 = USART_WordLength_9b | USART_Parity_Even | USART_Mode_Tx | USART_Mode_Rx;
		u.uart->CTLR2 = USART_StopBits_2;
		u.uart->CTLR3 = USART_HardwareFlowControl_None;
		uart_configure_baudrate(i, UART_BAUD_RATE_SAFE);
		// UART RX Interrupt (IDLE)
		u.uart->CTLR1 |= USART_CTLR1_IDLEIE;
		// UART RX Parity Error Interrupt (PEIE)
		u.uart->CTLR1 |= USART_CTLR1_PEIE;
		// Enable UART DMA TX & RX
		u.uart->CTLR3 |= USART_CTLR3_DMAT | USART_CTLR3_DMAR;
		// UART RX Error Interrupt (EIE)
		u.uart->CTLR3 |= USART_CTLR3_EIE;

		// Setup respective DMA Channel for UART RX
		DMA_CH(u.DMA, u.DMA_CH_RX)->CONTROL = 
			DMA_DIR_PeripheralSRC |
			DMA_Mode_Circular |
			DMA_PeripheralInc_Disable | DMA_MemoryInc_Enable |
			DMA_PeripheralDataSize_Byte | DMA_MemoryDataSize_Byte |
			DMA_Priority_Low;
		DMA_CH(u.DMA, u.DMA_CH_RX)->PER_ADDR = (uint32_t)(&u.uart->DATAR);
		DMA_CH(u.DMA, u.DMA_CH_RX)->MEM_ADDR = (uint32_t)UART_IO[i].rx_buffer;
		DMA_CH(u.DMA, u.DMA_CH_RX)->COUNTER = UART_RX_BUFFER_SIZE;

		// Setup respective DMA Channel for UART TX
		DMA_CH(u.DMA, u.DMA_CH_TX)->CONTROL = 
			DMA_DIR_PeripheralDST |
			DMA_Mode_Normal |
			DMA_PeripheralInc_Disable | DMA_MemoryInc_Enable |
			DMA_PeripheralDataSize_Byte | DMA_MemoryDataSize_Byte |
			DMA_Priority_Low;
		DMA_CH(u.DMA, u.DMA_CH_TX)->PER_ADDR = (uint32_t)(&u.uart->DATAR);
		// Enable TX Interrupt (TC)
		DMA_CH(u.DMA, u.DMA_CH_TX)->CONTROL |=	DMA_CFGR1_TCIE;
		// TODO on transfer
		/* DMA_CH(u.DMA, u.DMA_CH_TX)->MEM_ADDR = TX_ADDR;
		DMA_CH(u.DMA, u.DMA_CH_TX)->COUNTER = TX_LEN;
		DMA_CH(u.DMA, u.DMA_CH_TX)->CONTROL |= DMA_CFGR1_EN;*/

		NVIC_SetPriority(u.uartIRQ_RX, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 6, 1));
		NVIC_EnableIRQ(u.uartIRQ_RX);
		//NVIC_SetPriority(u.dmaIRQ_RX, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 6, 2));
		//NVIC_EnableIRQ(u.dmaIRQ_RX);
		NVIC_SetPriority(u.dmaIRQ_TX, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 6, 3));
		NVIC_EnableIRQ(u.dmaIRQ_TX);
	
		// Enable UART and DMA RX
		DMA_CH(u.DMA, u.DMA_CH_RX)->CONTROL |= DMA_CFGR1_EN;
		u.uart->CTLR1 |= USART_CTLR1_UE;
	}
}

/** Send data over UART port */
inline void uart_send_dma(uint_fast8_t port, const void* data, uint_fast16_t len)
{
	UART_STR("/TX:");
	UART_CHARR(UINT999_TO_CHARR(len));
	//while ((UART[port].uart->STATR & USART_STATR_TC) != USART_STATR_TC);
	DMA_CH(UART[port].DMA, UART[port].DMA_CH_TX)->MEM_ADDR = (uint32_t)data;
	DMA_CH(UART[port].DMA, UART[port].DMA_CH_TX)->COUNTER = len;
	DMA_CH(UART[port].DMA, UART[port].DMA_CH_TX)->CONTROL |= DMA_CFGR1_EN;
}

static inline bool uart_flush_TX(int port)
{
	DMA_CH(UART[port].DMA, UART[port].DMA_CH_TX)->CONTROL &= ~DMA_CFGR1_EN;
	//while (DMA_CH(UART[port].DMA, UART[port].DMA_CH_TX)->CONTROL & DMA_CFGR1_EN);
	__disable_irq();
	int p = UART_IO[port].tx_queue_pos;
	UART_IO[port].tx_queue_pos = (p + 1) % SZ_TX_QUEUE;
	if (UART_IO[port].tx_queue[p].valid)
	{
		uart_send_dma(port, (uint8_t*)UART_IO[port].tx_queue[p].addr, UART_IO[port].tx_queue[p].len);
		UART_STR("!QueueTX:");
		UART_CHARR(INT9_TO_CHARR(p));
		UART_IO[port].tx_queue[p].valid = false;
		UART_IO[port].uart_tx = true;
		__enable_irq();
		return true;
	}
	UART_IO[port].uart_tx = false;
	__enable_irq();
	return false;
}

/** USART global interrupt handlers */

static inline void UART_RX_Handler(uint8_t port)
{
	LOG_EVT_INT(CONTROLLER_INTERRUPT_UART, true);
	// Read stat & data register to clear flags
	uint8_t stat = UART[port].uart->STATR;
	uint32_t tmp = UART[port].uart->DATAR;
	(void) tmp;
	// Handle event
	if (stat & USART_STATR_IDLE)
	{
		// TODO: Consider calling from main loop. Less long blocking interrupts, but needs more care locking sections like debug
		uartd_process_port(port, DMA_TAIL(port));
	}
	if (stat & USART_STATR_NE)
	{
		UART_STR("+Noise");
		UART_CHARR(INT9_TO_CHARR(port));
	}
	if (stat & USART_STATR_FE)
	{
		UART_STR("+Frame");
		UART_CHARR(INT9_TO_CHARR(port));
	}
	if (stat & USART_STATR_PE)
	{
		UART_STR("+Parity");
		UART_CHARR(INT9_TO_CHARR(port));
	}
	LOG_EVT_INT(CONTROLLER_INTERRUPT_UART, false);
}
void USART1_IRQHandler() __IRQ;
void USART1_IRQHandler()
{ // UART IDLE
	UART_RX_Handler(0);
}
void USART2_IRQHandler() __IRQ;
void USART2_IRQHandler()
{ // UART IDLE
	UART_RX_Handler(1);
}
void USART3_IRQHandler() __IRQ;
void USART3_IRQHandler()
{ // UART IDLE
	UART_RX_Handler(2);
}
void UART4_IRQHandler() __IRQ;
void UART4_IRQHandler()
{ // UART IDLE
	UART_RX_Handler(3);
}
void UART5_IRQHandler() __IRQ;
void UART5_IRQHandler()
{ // UART IDLE
	UART_RX_Handler(4);
}
void UART6_IRQHandler() __IRQ;
void UART6_IRQHandler()
{ // UART IDLE
	UART_RX_Handler(5);
}
void UART7_IRQHandler() __IRQ;
void UART7_IRQHandler()
{ // UART IDLE
	UART_RX_Handler(6);
}
void UART8_IRQHandler() __IRQ;
void UART8_IRQHandler()
{ // UART IDLE
	UART_RX_Handler(7);
}
void DMA1_Channel2_IRQHandler() __IRQ;
void DMA1_Channel2_IRQHandler()
{ // UART3 TX Complete
	DMA1->INTFCR = DMA_GIF2;
	uart_flush_TX(2);
}
void DMA1_Channel4_IRQHandler() __IRQ;
void DMA1_Channel4_IRQHandler()
{ // UART1 TX Complete
	DMA1->INTFCR = DMA_GIF4;
	uart_flush_TX(0);
}
void DMA1_Channel7_IRQHandler() __IRQ;
void DMA1_Channel7_IRQHandler()
{ // UART2 TX Complete
	DMA1->INTFCR = DMA_GIF7;
	uart_flush_TX(1);
}
void DMA2_Channel4_IRQHandler() __IRQ;
void DMA2_Channel4_IRQHandler()
{ // UART5 TX Complete
	DMA2->INTFCR = DMA_GIF4;
	uart_flush_TX(4);
}
void DMA2_Channel5_IRQHandler() __IRQ;
void DMA2_Channel5_IRQHandler()
{ // UART4 TX Complete
	DMA2->INTFCR = DMA_GIF5;
	uart_flush_TX(3);
}
void DMA2_Channel6_IRQHandler() __IRQ;
void DMA2_Channel6_IRQHandler()
{ // UART6 TX Complete
	DMA2->INTFCR = DMA_GIF6;
	uart_flush_TX(5);
}
// Careful with channel 8,9,10,11 of DMA2
// Those use a separate status register DMA2_EXTEN (technically 8 is mirrored in DMA2 as well, but the defines can only index into DMA2_EXTEN)
void DMA2_Channel8_IRQHandler() __IRQ;
void DMA2_Channel8_IRQHandler()
{ // UART7 TX Complete
	DMA2_EXTEN->INTFCR = DMA_GIF8;
	uart_flush_TX(6);
}
void DMA2_Channel10_IRQHandler() __IRQ;
void DMA2_Channel10_IRQHandler()
{ // UART8 TX Complete
	DMA2_EXTEN->INTFCR = DMA_GIF10;
	uart_flush_TX(7);
}