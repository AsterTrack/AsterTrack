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

#include "config.h"
#include "pd_driver.h"
#include "power_control.h"
#include "util.h"

void Setup_Peripherals()
{
	// TODO: Enable/Disable debugging?
#ifdef DEBUG_FLASH
#else
#endif

	// -- Interrupt Controller --

	// Determine split of Preempt-Group priorities and Subpriorities in 4 bits of priority
	// CH32V307 can only do up to NVIC_PRIORITYGROUP_3 (8:2 split), CM3 can go up to NVIC_PRIORITYGROUP_4 (16:0)
	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_3); // 8 Preempt-Priorities, 2 Subpriorities

	// System interrupt init
	NVIC_SetPriority(EXC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));

	// -- System Clock --

	// Set to 144MHz in system_ch32v30x.c
	// Warning: UART Clocks also depend on this! Update SYSCLKFRQ in util.h if changed


	// -- System Timers --

	SysTick->CTLR = 1; // Enable, other default options are good (Tick Frequency = SysClk / 8 = 18MHz)

	{ // TIM3 is used for SOF/Sync timer
		// Set Timer interrupts
		NVIC_SetPriority(TIM3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 4, 0));
		NVIC_EnableIRQ(TIM3_IRQn);

		// Initialise Timer
		RCC->APB1PRSTR |= RCC_APB1Periph_TIM3; // Reset Timer
		RCC->APB1PRSTR &= ~RCC_APB1Periph_TIM3;
		RCC->APB1PCENR |= RCC_APB1Periph_TIM3; // Enable internal clock for Timer

		// Setup TIM3 for 1us counter
		TIM3->PSC = SYSCLKFRQ-1; // Set Prescaler to get a 1MHz timer (1us interval)
		TIM3->CTLR1 = TIM_ARPE | TIM_URS; // Enable Auto-Reload Preload, and only generate update on overflow
	}

	{ // TIM4 is used for BOOT/Flash button response
		// Set Timer interrupts
		NVIC_SetPriority(TIM4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
		NVIC_EnableIRQ(TIM4_IRQn);

		// Initialise Timer
		RCC->APB1PRSTR |= RCC_APB1Periph_TIM4; // Reset Timer
		RCC->APB1PRSTR &= ~RCC_APB1Periph_TIM4;
		RCC->APB1PCENR |= RCC_APB1Periph_TIM4; // Enable internal clock for Timer

		// Setup TIM4 for 100us counter
		TIM4->PSC = (SYSCLKFRQ*100)-1; // Set Prescaler to get a 10KHz timer (100us interval)
		TIM4->CTLR1 = TIM_ARPE | TIM_URS; // Enable Auto-Reload Preload, and only generate update on overflow
	}

	{ // TIM6 is used for SPI2 LED Latch set after sending
		// Set Timer interrupts
		NVIC_SetPriority(TIM6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 7, 0));
		NVIC_EnableIRQ(TIM6_IRQn);

		// Initialise Timer
		RCC->APB1PRSTR |= RCC_APB1Periph_TIM6; // Reset Timer
		RCC->APB1PRSTR &= ~RCC_APB1Periph_TIM6;
		RCC->APB1PCENR |= RCC_APB1Periph_TIM6; // Enable internal clock for Timer

		// Setup TIM6 for 10us counter
		TIM6->PSC = (SYSCLKFRQ*10)-1; // Set Prescaler to get a 100KHz timer (10us interval)
		TIM6->CTLR1 = TIM_ARPE | TIM_URS; // Enable Auto-Reload Preload, and only generate update on overflow
	}


	// -- GPIO --

	// Peripheral clock enable
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE;

	// LED on most boards
	GPIO_CFG(GPIOC, GPIO_PIN_13, GPIO_PP_OUT | GPIO_Speed_2MHz);
	

	// -- SYNC lines --

	// Sync Out to cameras
	GPIO_CFG(GPIOD, GPIO_PIN_8, GPIO_PP_OUT | GPIO_Speed_10MHz);
	GPIO_CFG(GPIOD, GPIO_PIN_9, GPIO_PP_OUT | GPIO_Speed_10MHz);
	GPIO_CFG(GPIOD, GPIO_PIN_10, GPIO_PP_OUT | GPIO_Speed_10MHz);
	GPIO_CFG(GPIOD, GPIO_PIN_11, GPIO_PP_OUT | GPIO_Speed_10MHz);
	GPIO_CFG(GPIOD, GPIO_PIN_12, GPIO_PP_OUT | GPIO_Speed_10MHz);
	GPIO_CFG(GPIOD, GPIO_PIN_13, GPIO_PP_OUT | GPIO_Speed_10MHz);
	GPIO_CFG(GPIOD, GPIO_PIN_14, GPIO_PP_OUT | GPIO_Speed_10MHz);
	GPIO_CFG(GPIOD, GPIO_PIN_15, GPIO_PP_OUT | GPIO_Speed_10MHz);

	// Sync IO Direction output
	GPIO_CFG(GPIOE, GPIO_PIN_2, GPIO_PP_OUT | GPIO_Speed_10MHz);

	// Set Transceiver Receiver Enable for RX 
	GPIO_RESET(GPIOE, GPIO_PIN_2);

	// Rest for Sync IO is setup on-demand
	GPIO_CFG(GPIOE, GPIO_PIN_3, GPIO_FLOATING_IN);


	// -- POWER CONTROL --

	// Setup Power Input toggles
	GPIO_CFG(GPIOE, GPIO_PIN_7, GPIO_PP_OUT | GPIO_Speed_2MHz); // External Power I/O
	GPIO_CFG(GPIOE, GPIO_PIN_8, GPIO_PP_OUT | GPIO_Speed_2MHz); // USB-C PD Power In

	DisablePowerIn();

	// Setup Power Voltage Reads

	// Reset ADC
	RCC->APB2PRSTR |= RCC_APB2Periph_ADC1;
	RCC->APB2PRSTR &= ~RCC_APB2Periph_ADC1;
	RCC->APB2PRSTR |= RCC_APB2Periph_ADC2;
	RCC->APB2PRSTR &= ~RCC_APB2Periph_ADC2;

	// RM says maximum ADC clock is 14Mhz.
	// So can't operate with PCLK2 at 144MHz because ADCPRE has a max divider of 8, and 18Mhz > 14Mhz
	// But, one chinese forum post mentions it supports up to 36Mhz, so surely they're right!
	// https://bbs.21ic.com/icview-3481199-1-1.html
	// Lowering PCKL2 requires running UART1 at 4MBaud, or running the whole CPU at 112Mhz and ALL UARTs at 7MBaud
	// Since these drawbacks are significant, we're instead running the ADC at 18Mhz and accepting any oddities:
	// So far observed only one anomaly: Regular Channel 1 does not apply the Sample Time reliably
	// No matter which channel I map Regular Channel 1 to, it seems to use a sample time of 0 (11+0 ADCCLK) half the time
	// This results in very unreliable readings, making Regular Channel 1 useless
	// Regular Channel 0 is unaffected, other channels have not been tested, only ADC1 has been tested
	// So since we only need two channels anyway, we use both ADC1 and ADC2 with 1 channel each

	// Setup ADC Clocks
	RCC->CFGR0 = (RCC->CFGR0 & ~RCC_ADCPRE) | RCC_ADCPRE_DIV8;
	RCC->APB2PCENR |= RCC_APB2Periph_ADC1;
	RCC->APB2PCENR |= RCC_APB2Periph_ADC2;

	// Configure ADC
	ADC1->CTLR1 = 0; // Just one channel to measure
	ADC1->CTLR2 = ADC_CONT; // Enable continuous conversion
	ADC2->CTLR1 = 0; // Just one channel to measure
	ADC2->CTLR2 = ADC_CONT; // Enable continuous conversion

	// Configure ADC channels
	ADC1->SAMPTR2 = 0b101 << 27; // Select 11+55.5 ADCCLK Sample Time for Channel 9
	ADC1->RSQR1 = 1 << 20; // Set number of regular channels to 1
	ADC1->RSQR3 = (9 << 0); // Set as Input 9 (PD)
	ADC2->SAMPTR2 = 0b101 << 24; // Select 11+55.5 ADCCLK Sample Time for Channel 8
	ADC2->RSQR1 = 1 << 20; // Set number of regular channels to 1
	ADC2->RSQR3 = (8 << 0); // Set Input 8 (Ext)
	// -> Timings result in both ADCs sampling every 3.7us (11+55.5 CLK @ 18Mhz)

	// Enable IRQ for ADC for Analog Watchdog
	// High priority since it might be a spike in voltage that requires quickly cutting power
	NVIC_SetPriority(ADC1_2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));
	NVIC_EnableIRQ(ADC1_2_IRQn);


	// -- RESET/FLASH BUTTON --

	// Enable AFIO and EXTI clock
	RCC->APB2PCENR |= RCC_APB2Periph_AFIO;

	GPIO_CFG(GPIOE, GPIO_PIN_1, GPIO_FLOATING_IN);
	// has external 10K Ohm pulldown resistor

	// Setup NVIC interrupt handlers for EXTI line
	NVIC_SetPriority(EXTI1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
	NVIC_EnableIRQ(EXTI1_IRQn);

	// Select port E for EXTI line
	AFIO->EXTICR[0] = (AFIO->EXTICR[0] & ~AFIO_EXTICR1_EXTI1) | AFIO_EXTICR1_EXTI1_PE;

	// Setup interrupts for EXTI line
	EXTI->RTENR |= EXTI_LINE_1; // Set to trigger on rising edge
	EXTI->INTENR |= EXTI_LINE_1; // Enable interrupt generation

	// Set BOOT1 to 0 (for booting into bootloader instead of SRAM when resetting with BOOT0=1)
	GPIO_CFG(GPIOB, GPIO_PIN_2, GPIO_PP_OUT | GPIO_Speed_2MHz);
	GPIO_RESET(GPIOB, GPIO_PIN_2);


	// -- SPI LED CONTROL --

	// Setup SPI output pins
	GPIO_CFG(GPIOB, GPIO_PIN_12, GPIO_PP_OUT | GPIO_Speed_2MHz); // Latch
	GPIO_CFG(GPIOB, GPIO_PIN_13, GPIO_AF_PP_OUT | GPIO_Speed_10MHz); // Clock
	GPIO_CFG(GPIOB, GPIO_PIN_15, GPIO_AF_PP_OUT | GPIO_Speed_10MHz); // MOSI

	// Reset SPI2
	RCC->APB1PRSTR |= RCC_APB1Periph_SPI2;
	RCC->APB1PRSTR &= ~RCC_APB1Periph_SPI2;

	// Enable SPI2 clock
	RCC->APB1PCENR |= RCC_APB1Periph_SPI2;

	// SPI target: 2 chained 595 latched shift register
	SPI2->CTLR1 = SPI_CTLR1_DFF | SPI_CTLR1_SSM | SPI_CTLR1_SSI | (SPI_CTLR1_BR_2 | SPI_CTLR1_BR_0);
	// DFF for 16bit data so both 595s get written to at once
	// CPOL=0, CPHA=0 so that clock is default 0, and data is sent with a rising edge
	// NSS is the latch pin, so it's detached from the physical pin since NSS has some requirements for SPI hardware
	// Instead, NSS will be always high (SSI) and Latch will be controller using normal GPIO (not Alternate Function)
	// BR=101 for a 64 divider (should be 2.25MHz, should be well below the minimum of the shift register even at 2V)
	
	// TXE interrupt occurs after the data is out of DATAR ready to be sent, NOT when the data has been sent
	// So, use TIM6 to trigger latch after data has been sent, but use TXE to get exactly when the data has been submitted for sending
	// Except SPI interrupt also does not work, I probably can't clear the IRQ, so it locks up the system

	// Setup NVIC interrupt handlers for SPI2
	/* NVIC_SetPriority(SPI2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 7, 0));
	NVIC_EnableIRQ(SPI2_IRQn);

	// Enable SPI2 TXE interrupt
	SPI2->CTLR2 = SPI_CTLR2_TXEIE; */

	// Enable SPI2 in master mode
	SPI2->CTLR1 |= SPI_CTLR1_SPE | SPI_CTLR1_MSTR;


	// -- POWER DELIVERY --

	// Setup IRQ pin

	GPIO_CFG(GPIOB, GPIO_PIN_5, GPIO_PU_PD_IN);
	GPIO_SET(GPIOB, GPIO_PIN_5); // Enable Pull-Up

	// Setup NVIC interrupt handlers for EXTI line
	NVIC_SetPriority(EXTI9_5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 7, 0));
	NVIC_DisableIRQ(EXTI9_5_IRQn);
	// Needs to be interruptable since PD driver can wait dozens of milliseconds

	// Select port B for EXTI line
	AFIO->EXTICR[1] = (AFIO->EXTICR[1] & ~AFIO_EXTICR2_EXTI5) | AFIO_EXTICR2_EXTI5_PB;

	// Setup interrupts for EXTI line
	EXTI->FTENR |= EXTI_LINE_5; // Set to trigger on falling edge
	EXTI->INTENR |= EXTI_LINE_5; // Enable interrupt generation


	// Setup I2C

	GPIO_CFG(GPIOB, GPIO_PIN_8, GPIO_AF_OD_OUT | GPIO_Speed_50MHz);
	GPIO_CFG(GPIOB, GPIO_PIN_9, GPIO_AF_OD_OUT | GPIO_Speed_50MHz);

	// Reset IC21
	RCC->APB1PRSTR |= RCC_APB1Periph_I2C1;
	RCC->APB1PRSTR &= ~RCC_APB1Periph_I2C1;

	// Enable I2C1 clock
	RCC->APB1PCENR |= RCC_APB1Periph_I2C1;

	// Enable I2C1 remap to PB8 & PB9
	AFIO->PCFR1 |= AFIO_PCFR1_I2C1_REMAP;

	uint32_t srcClk = PCLK1;
	uint32_t srcClkMhz = PCLK1/1000000;
	uint32_t srcClkMax = srcClkMhz;
	// Register max is 36, but takes 63 in terms of bits.
	if (srcClkMax > 36)
		srcClkMax = 36;
	const int i2cClkRate = 400000;
	if (i2cClkRate <= 100000)
	{ // <= 100kHz
		uint16_t i2cClkDiv = srcClk / (i2cClkRate << 1);
		// 180 for 36MHz, 720 for 144MHz
		if (i2cClkDiv < 4)
			i2cClkDiv = 4;
		I2C1->CKCFGR = i2cClkDiv & I2C_CKCFGR_CCR; // ClkDiv max: 2047
		I2C1->RTR = (srcClkMhz+1) & I2C_RTR_TRISE; // TRISE max: 63
		I2C1->CTLR2 = srcClkMax & I2C_CTLR2_FREQ; // PCKL1 FREQ max: 36. Really.
	}
	else
	{ // 100-400Khz
		I2C1->CKCFGR = I2C_CKCFGR_FS;
		uint16_t i2cClkDiv;
		if (true)
		{ // 2:1 duty cycle
			i2cClkDiv = srcClk / (i2cClkRate * 3);
			// 30 for 36MHz, 120 for 144MHz
		}
		else
		{ // 16:9 duty cycle
			I2C1->CKCFGR |= I2C_CKCFGR_DUTY;
			i2cClkDiv = srcClk / (i2cClkRate * 25);
			// 3(3.6) for 36MHz, 14(14.4) for 144MHz
		}
		if ((i2cClkDiv & I2C_CKCFGR_CCR) == 0)
			i2cClkDiv = 1;
		I2C1->CKCFGR |= i2cClkDiv & I2C_CKCFGR_CCR; // ClkDiv max: 2047
		I2C1->RTR = (srcClkMhz * 300 / 1000 + 1) & I2C_RTR_TRISE; // TRISE max: 63
		// 12(11.8) for 36MHz, 45(44.2) for 144MHz
		I2C1->CTLR2 = srcClkMax; // PCKL1 FREQ max: 36Mhz
		// But can also just write 36 even when actually at 144, no immediate problem
		// However I suspect this will affect internal timeouts
	}

	pd_init(); // This takes dozens of ms due to synchronous I2C and timing
	// Enable interrupt only after initialisation
	NVIC_EnableIRQ(EXTI9_5_IRQn);
}


/**
 * Sync port setup
 */

void SYNC_Output_Init()
{
	// Setup Data Pin for TX
	GPIO_CFG(GPIOE, GPIO_PIN_3, GPIO_PP_OUT | GPIO_Speed_2MHz);

	// Set Transceiver Driver Enable for TX 
	GPIO_SET(GPIOE, GPIO_PIN_2);
}

void SYNC_Input_Init()
{
	// Setup Data Pin for RX
	GPIO_CFG(GPIOE, GPIO_PIN_3, GPIO_PU_PD_IN);
	GPIO_RESET(GPIOE, GPIO_PIN_3); // Enable Pull-Down

	// Set Transceiver Receiver Enable for RX 
	GPIO_RESET(GPIOE, GPIO_PIN_2);

	// Setup NVIC interrupt handlers for EXTI line
	NVIC_SetPriority(EXTI3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 4, 0)); // Same priority as SOF timer
	NVIC_EnableIRQ(EXTI3_IRQn);

	// Select port E for EXTI line
	AFIO->EXTICR[0] = (AFIO->EXTICR[0] & ~AFIO_EXTICR1_EXTI3) | AFIO_EXTICR1_EXTI3_PE;

	// Setup interrupts for EXTI line
	EXTI->RTENR |= GPIOE_SYNC_EXTI_LINES; // Set to trigger on rising edge
	EXTI->INTENR |= GPIOE_SYNC_EXTI_LINES; // Enable interrupt generation
}

void SYNC_Reset()
{
	GPIO_CFG(GPIOE, GPIO_PIN_3, GPIO_FLOATING_IN);
	NVIC_DisableIRQ(EXTI3_IRQn);
	// Reset interrupts for EXTI line
	EXTI->INTENR &= ~GPIOE_SYNC_EXTI_LINES; // Disable interrupt generation
	EXTI->INTFR = GPIOE_SYNC_EXTI_LINES; // Clear pending interrupt
}


/**
 * General IRQ Handlers
 */


/**
 * Reset/Flash button pressed
 */
void EXTI1_IRQHandler(void) __IRQ;
void EXTI1_IRQHandler()
{
	LOG_EVT_INT(CONTROLLER_INTERRUPT_FLASH_BUTTON, true);
	if (EXTI->INTFR & EXTI_LINE_1)
	{ // Interrupt pending

		ERR_STR("/RESET"); // what kind (normal, bootloader) is still not determined

		// Set to wait 500ms to check if button is still pressed
		StartTimer(TIM4, 5000); // 100us intervals
	}

	// Reset IRQ flag
	EXTI->INTFR = EXTI_LINE_1;
	LOG_EVT_INT(CONTROLLER_INTERRUPT_FLASH_BUTTON, false);
}

/**
 * Act on earlier Reset/Flash button press
 */
void TIM4_IRQHandler(void) __IRQ;
void TIM4_IRQHandler()
{
	LOG_EVT_INT(CONTROLLER_INTERRUPT_FLASH_TIMER, true);
	StopTimer(TIM4);
	TIM_SR(TIM4) = 0;

	// If BOOT0 is still pressed, it will reboot into bootloader
	// If not, it will reset normally
	//bool longPress = GPIO_READ(GPIOE, GPIO_PIN_1);

	// System Reset
	PFIC->CFGR = NVIC_KEY3 | 0x0080;
	LOG_EVT_INT(CONTROLLER_INTERRUPT_FLASH_BUTTON, false);
}

/**
 * SPI2 TXE interrupt, so data is starting to be sent
 */
/* void SPI2_IRQHandler(void) __IRQ;
void SPI2_IRQHandler()
{ // Doesn't work, not used rn
	// Takes about 7us to send at 144/64 = 2.25Mhz, so wait 10us
	StartTimer(TIM6, 100);
	NVIC_ClearPendingIRQ(SPI2_IRQn);
} */

/**
 * FUSB302 USB-C PD Interrupt
 */
void EXTI9_5_IRQHandler(void) __IRQ;
void EXTI9_5_IRQHandler()
{
	LOG_EVT_INT(CONTROLLER_INTERRUPT_PD_INT, true);
	if (EXTI->INTFR & EXTI_LINE_5)
	{ // Interrupt pending
		EXTI->INTFR = EXTI_LINE_5;
		pd_poll();
	}
	LOG_EVT_INT(CONTROLLER_INTERRUPT_PD_INT, false);
}

void ErrorTrap(void)
{
	// Lock up, flicker LEDs, handle reset
	uint32_t loop = 0;
	while (1)
	{
		if (GPIO_READ(GPIOE, GPIO_PIN_1))
		{ // Still allow the reset button to reset and enter flashing mode
			delayUS(500000);
			// System Reset
			PFIC->CFGR = NVIC_KEY3 | 0x0080;
			// If BOOT0 is still pressed, it will reboot into bootloader
		}

		loop++;
		if (loop % 1000 == 0)
		{ // Toggle all LEDs between ON/OFF state every 100ms
			SPI2->DATAR = loop%2000 == 0? 0x0000 : 0xFFFF;
			delayUS(20); // 20us, Write time until Latch
			GPIO_SET(GPIOB, GPIO_PIN_12);
			delayUS(20); // 20us, Latch pulse width
			GPIO_RESET(GPIOB, GPIO_PIN_12);
		}

		delayUS(100);
	}
}

// Exception PC (program counter) is stored in separate register mepc
// GDB doesn't read it unfortunately, and only uses pc (Handler) and ra (Function calling the function that faults)
// So to get the faulting address, need to set ra to mepc.
// (naked) preserves the original sp (stack pointer) of the faulty function (making it execute JUST the following two instructions on a trap)
// __IRQ (interrupt("machine")), which adds to the stack for the handler, also works it seems


void Break_Point_Handler(void) __attribute__((naked));
void Break_Point_Handler(void)
{
}

void HardFault_Handler(void) __attribute__((naked));
void HardFault_Handler(void)
{
	__asm(
		//"csrr t0, mcause;"
		"mv t6, ra;"
		"csrr ra, mepc;"
		"EBREAK;"
	);
	ErrorTrap();
}

void NMI_Handler(void) __attribute__((naked));
void NMI_Handler(void)
{
	__asm(
		//"csrr t0, mcause;"
		"mv t6, ra;"
		"csrr ra, mepc;"
		"EBREAK;"
	);
	ErrorTrap();
}

void Ecall_M_Mode_Handler(void) __attribute__((naked));
void Ecall_M_Mode_Handler(void)
{
	__asm(
		//"csrr t0, mcause;"
		"mv t6, ra;"
		"csrr ra, mepc;"
		"EBREAK;"
	);
	ErrorTrap();
}

void Ecall_U_Mode_Handler(void) __attribute__((naked));
void Ecall_U_Mode_Handler(void)
{
	__asm(
		//"csrr t0, mcause;"
		"mv t6, ra;"
		"csrr ra, mepc;"
		"EBREAK;"
	);
	ErrorTrap();
}

void WWDG_IRQHandler(void) __attribute__((naked));
void WWDG_IRQHandler(void)
{
	//WWDG->CTLR = (WWDG_TIMEOUT & WWDG_CTLR_T);
	WWDG->STATR = 0;
	__asm(
		//"csrr t0, mcause;"
		"mv t6, ra;"
		"csrr ra, mepc;"
		"EBREAK;"
	);
	ErrorTrap();
}
