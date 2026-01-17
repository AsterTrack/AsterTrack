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
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_system.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_wwdg.h"
//#include "stm32g0xx_ll_pwr.h"
//#include "stm32g0xx_ll_exti.h"
//#include "stm32g0xx_ll_tim.h"
#endif

#include "config_impl.h"
#include "util.h"

// Global System Timer
volatile uint64_t usCounter;

#define TIM1_ARR 65000

void SystemInit(void) {}

void Setup_Peripherals()
{
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

	// -- Interrupt Controller --

	// System interrupt init
	// Only 2 bits of preempt priority, no sub-priorities, so PriorityGrouping doesn't matter (== 0)
	NVIC_SetPriority(SVCall_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_SetPriority(PendSV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));


	// -- System Clock --

#if defined(STM32G0)

	// Configure to 64Mhz
	// USART needs 64Mhz source to get to maximum frequency (8Mhz with 64MHz / 8 with fast sampling)
	// Cannot use PLL directly, so need SYSCLK(or PCLK which may be further divided)
	// So forced do use SYSCLK of 64MHz

	// Configure HSI16
	if (!LL_RCC_HSI_IsReady())
		LL_RCC_HSI_Enable();
	while (!LL_RCC_HSI_IsReady());

	// Set Flash Latency for 64MHz
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

	// Configure PLL to 64MHz for both SYSCLK and ADC
	// TODO: Can set ADC clock up to 122Mhz with this
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 8, LL_RCC_PLLR_DIV_2);
	//LL_RCC_PLL_ConfigDomain_ADC(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 8, LL_RCC_PLLP_DIV_8);
	LL_RCC_PLL_Enable();
	LL_RCC_PLL_EnableDomain_SYS();
	//LL_RCC_PLL_EnableDomain_ADC();
	while (!LL_RCC_PLL_IsReady());

	// Set system clock
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1); // HCLK 64MHz
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1); // PCLK 64MHz
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL); // SYSCLK 64MHz
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

	//assert(SYSCLKFRQ == 64);
#endif

	// Warning: UART Clocks also depend on this! Update SYSCLKFRQ in util.h if changed here

	// -- System Timers --

	{
		// Set TIM1 interrupts
		NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
		NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);

		// Initialise TIM1
		TIM1->CR1 &= ~TIM_CR1_CEN; // Disable Counter
		RCC->APBRSTR2 |= RCC_APBRSTR2_TIM1RST; // Reset TIM1
		RCC->APBRSTR2 &= ~RCC_APBRSTR2_TIM1RST;
		RCC->APBENR2 |= RCC_APBENR2_TIM1EN; // Enable internal clock for TIM1

		// Setup TIM1 for 1us counter	
		TIM1->PSC = SYSCLKFRQ-1; // Set Prescaler to get a 1MHz timer (1us interval)
		TIM1->ARR = TIM1_ARR-1; // Set Auto-Reload to count up a known value, and use the interrupt to keep a 64-bit timer valid
		TIM1->CR1 |= TIM_CR1_ARPE; // Enable Auto-Reload Preload
		TIM1->EGR |= TIM_EGR_UG; // Trigger update event to load values
		
		// Start TIM1
		TIM1->DIER = TIM_DIER_UIE; // Enable update interrupt
		TIM1->CR1 |= TIM_CR1_CEN; // Enable Counter
		
		// Now TIM1_IRQHandler is called every TIM1_ARR ms, for us to update a 64Bit counter
		// Additionally, number of us within that 1ms frame can be read from TIM1->CNT
	}

	// -- GPIO --

	// Peripheral clock enable
#if defined(STM32G0)
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
#endif

	// RJ45 LEDs
	LL_GPIO_SetPinMode(RJLED_GPIO_X, RJLED_GREEN_PIN, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(RJLED_GPIO_X, RJLED_GREEN_PIN, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(RJLED_GPIO_X, RJLED_GREEN_PIN, LL_GPIO_SPEED_FREQ_LOW);
	LL_GPIO_SetPinMode(RJLED_GPIO_X, RJLED_ORANGE_PIN, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(RJLED_GPIO_X, RJLED_ORANGE_PIN, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(RJLED_GPIO_X, RJLED_ORANGE_PIN, LL_GPIO_SPEED_FREQ_LOW);

	// UART Select Output
	LL_GPIO_SetPinMode(UARTSEL_GPIO_X, UARTSEL_PIN, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(UARTSEL_GPIO_X, UARTSEL_PIN, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(UARTSEL_GPIO_X, UARTSEL_PIN, LL_GPIO_SPEED_FREQ_LOW);

	// Filter Switcher Actuators
	LL_GPIO_SetPinMode(FILTERSW_GPIO_X, FILTERSW_INFRARED_PIN, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(FILTERSW_GPIO_X, FILTERSW_INFRARED_PIN, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(FILTERSW_GPIO_X, FILTERSW_INFRARED_PIN, LL_GPIO_SPEED_FREQ_LOW);
	LL_GPIO_SetPinMode(FILTERSW_GPIO_X, FILTERSW_VISIBLE_PIN, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(FILTERSW_GPIO_X, FILTERSW_VISIBLE_PIN, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(FILTERSW_GPIO_X, FILTERSW_VISIBLE_PIN, LL_GPIO_SPEED_FREQ_LOW);
	LL_GPIO_SetPinMode(FILTERSW_GPIO_X, FILTERSW_PIN_SLEEP, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(FILTERSW_GPIO_X, FILTERSW_PIN_SLEEP, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(FILTERSW_GPIO_X, FILTERSW_PIN_SLEEP, LL_GPIO_SPEED_FREQ_LOW);

	// Sync input
	LL_GPIO_SetPinMode(SYNC_GPIO_X, SYNC_PIN, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinPull(SYNC_GPIO_X, SYNC_PIN, LL_GPIO_PULL_DOWN);
	LL_GPIO_SetPinSpeed(SYNC_GPIO_X, SYNC_PIN, LL_GPIO_SPEED_FREQ_HIGH);

	// Camera FSIN output
	LL_GPIO_SetPinMode(FSIN_GPIO_X, CAMERA_FSIN_PIN, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(FSIN_GPIO_X, CAMERA_FSIN_PIN, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(FSIN_GPIO_X, CAMERA_FSIN_PIN, LL_GPIO_SPEED_FREQ_HIGH);

	// Camera STROBE input
	LL_GPIO_SetPinMode(STROBE_GPIO_X, CAMERA_STROBE_PIN, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinPull(STROBE_GPIO_X, CAMERA_STROBE_PIN, LL_GPIO_PULL_DOWN);
	LL_GPIO_SetPinSpeed(STROBE_GPIO_X, CAMERA_STROBE_PIN, LL_GPIO_SPEED_FREQ_HIGH);

	// I2C/General SBC INT output
	LL_GPIO_SetPinMode(I2C_INT_GPIO_X, I2C_INT_PIN, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(I2C_INT_GPIO_X, I2C_INT_PIN, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(I2C_INT_GPIO_X, I2C_INT_PIN, LL_GPIO_SPEED_FREQ_LOW);

	// Button Inputs
	LL_GPIO_SetPinMode(BUTTONS_GPIO_X, BUTTON_BOTTOM_PIN, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinPull(BUTTONS_GPIO_X, BUTTON_BOTTOM_PIN, LL_GPIO_PULL_UP);
	LL_GPIO_SetPinSpeed(BUTTONS_GPIO_X, BUTTON_BOTTOM_PIN, LL_GPIO_SPEED_FREQ_LOW);
	LL_GPIO_SetPinMode(BUTTONS_GPIO_X, BUTTON_TOP_PIN, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinPull(BUTTONS_GPIO_X, BUTTON_TOP_PIN, LL_GPIO_PULL_UP);
	LL_GPIO_SetPinSpeed(BUTTONS_GPIO_X, BUTTON_TOP_PIN, LL_GPIO_SPEED_FREQ_LOW);

	// VSense ADC Pin
	LL_GPIO_SetPinMode(VSENSE_GPIO_X, VSENSE_ADC_PIN, LL_GPIO_MODE_ANALOG);


	// -- EXTI --

#if defined(BOARD_OLD) && defined(USE_SYNC)
	if (SYNC_PIN == GPIO_PIN_9 && SYNC_GPIO_X == GPIOB)
	{
		// Setup NVIC interrupt handler for EXTI line 9
		NVIC_SetPriority(EXTI4_15_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0));
		NVIC_EnableIRQ(EXTI4_15_IRQn);

		// Select PB (0x01) for EXTI Line 9 - so PB9
		EXTI->EXTICR[2] = (EXTI->EXTICR[2] & ~EXTI_EXTICR3_EXTI9_Msk) | (0x01 << EXTI_EXTICR3_EXTI9_Pos);

		// Setup interrupts for EXTI line 9
		EXTI->RTSR1 |= LL_EXTI_LINE_9; // Set to trigger on rising edge
		EXTI->IMR1 |= LL_EXTI_LINE_9; // Enable interrupt generation
	}
#endif
}

void EXTI4_15_IRQHandler(void) __IRQ;
void EXTI4_15_IRQHandler()
{
#if defined(BOARD_OLD) && defined(USE_SYNC)
	if (EXTI->RPR1 & LL_EXTI_LINE_9)
	{ // Interrupt pending

		GPIO_SET(FSIN_GPIO_X, CAMERA_FSIN_PIN);
		delayUS(FSIN_PULSE_WIDTH_US);
		GPIO_RESET(FSIN_GPIO_X, CAMERA_FSIN_PIN);

		// Reset IRQ flag
		EXTI->RPR1 = LL_EXTI_LINE_9;
	}
#endif
}

enum CameraMCUFlashConfig ReadFlashConfiguration()
{
	uint32_t optReg = FLASH->OPTR;
	if (FLASH->SR & FLASH_SR_OPTVERR)
		return MCU_FLASH_ERROR;
	else if ((optReg & FLASH_OPTR_nBOOT1) && !(optReg & FLASH_OPTR_nBOOT_SEL))
		return MCU_FLASH_BOOT0_PI;
	else if ((optReg & FLASH_OPTR_nBOOT1) && (optReg & FLASH_OPTR_nBOOT_SEL))
		return MCU_FLASH_DEBUG_SWD;
	else
		return MCU_FLASH_UNKNOWN;
}

#define FLASH_SR_ERRORS (FLASH_SR_OPERR  | FLASH_SR_PROGERR | FLASH_SR_WRPERR | \
						 FLASH_SR_PGAERR | FLASH_SR_SIZERR  | FLASH_SR_PGSERR | \
						 FLASH_SR_MISERR | FLASH_SR_FASTERR | \
						 FLASH_SR_OPTVERR)

static bool UnlockFlash()
{
	if (FLASH->CR & FLASH_CR_LOCK)
	{
		while (FLASH->SR & FLASH_SR_BSY1);

		// Unlock FLASH->CR
		FLASH->KEYR = 0x45670123U;
		FLASH->KEYR = 0xCDEF89ABU;

		if (FLASH->CR & FLASH_CR_LOCK)
			return false;
	}
	return true;
}

enum CameraMCUFlashConfig SetFlashConfiguration(enum CameraMCUFlashConfig config)
{
	enum CameraMCUFlashConfig currentConfig = ReadFlashConfiguration();
	if (config == MCU_FLASH_KEEP || currentConfig == config)
	{ // No need to configure
		// Lock Flash and Options for this boot
		FLASH->CR = 0xC0000000 | FLASH_CR_LOCK | FLASH_CR_OPTLOCK;
		return MCU_FLASH_KEEP;
	}

	if (config != MCU_FLASH_DEBUG_SWD && config != MCU_FLASH_BOOT0_PI)
		return MCU_FLASH_KEEP;

	// Update configuration

	if (!UnlockFlash())
		return MCU_FLASH_ERROR;

	if (FLASH->CR & FLASH_CR_OPTLOCK)
	{
		// Unlock Options
		FLASH->OPTKEYR = 0x08192A3BU;
		FLASH->OPTKEYR = 0x4C5D6E7FU;

		if (FLASH->CR & FLASH_CR_OPTLOCK)
			return MCU_FLASH_ERROR;
	}

	__disable_irq();

	while (FLASH->SR & FLASH_SR_BSY1);

	// Clear error status
	FLASH->SR |= FLASH_SR_ERRORS | FLASH_SR_EOP;

	// Default OPTR: 0xDFFFE1AA
	if (config == MCU_FLASH_DEBUG_SWD)
	{
		// Set Default:
		// - nBOOT1 = 1		(DC, we don't use bootloader)
		// - nBOOT_SEL = 1	(use nBOOT0 bit instead of BOOT0 pin)
		// - nBOOT0 = 1		(boot into main flash)
		// Always boot into Main Flash, BOOT0 pin is not used - so SWD works and can flash (BOOT0 and SWCLK share pin PA14)
		FLASH->OPTR = 0xDFFFE1AA;
	}
	else if (config == MCU_FLASH_BOOT0_PI)
	{
		// Modify:
		// - nBOOT1 = 1		(use system memory as bootloader IF BOOT0 is high)
		// - nBOOT_SEL = 0	(use BOOT0 pin instead of nBOOT0 bit)
		// - nBOOT0 = 1		(DC, disabled)
		// BOOT0 pin determines whether to enter bootloader - so Pi can flash, but SWD doesn't (BOOT0 and SWCLK share pin PA14)
		FLASH->OPTR = 0xDEFFE1AA;
	}

	// Write option byte to flash
	FLASH->CR |= FLASH_CR_OPTSTRT;

	// Wait for operation to finish
	while (FLASH->SR & FLASH_SR_BSY1);

	// Read and clear error status
	uint16_t error = FLASH->SR & FLASH_SR_ERRORS;
	FLASH->SR |= FLASH_SR_ERRORS | FLASH_SR_EOP;

	// Lock Options and FLASH->CR again
	FLASH->CR = FLASH_CR_LOCK | FLASH_CR_OPTLOCK;

	__enable_irq();

	if (error)
		return MCU_FLASH_ERROR;

	return config;
}

__attribute__((section(".RamFunc"), noinline, optimize("O0")))
uint16_t EraseFlashPage(uint16_t page)
{
	if (!UnlockFlash())
		return 1;

	__disable_irq();

	while (FLASH->SR & FLASH_SR_BSY1);

	// Clear error status
	FLASH->SR |= FLASH_SR_ERRORS | FLASH_SR_EOP;

	// Erase page in flash
	FLASH->CR &= ~FLASH_CR_PNB_Msk;
	FLASH->CR |= FLASH_CR_PER | ((page << FLASH_CR_PNB_Pos) & FLASH_CR_PNB_Msk);
	FLASH->CR |= FLASH_CR_STRT;

	// Wait for operation to finish
	while (FLASH->SR & FLASH_SR_BSY1);

	// Clear flash erase flag
	FLASH->CR &= ~(FLASH_CR_PER | FLASH_CR_PNB_Msk);

	// Read and clear error status
	uint16_t error = FLASH->SR & FLASH_SR_ERRORS;
	FLASH->SR |= FLASH_SR_ERRORS | FLASH_SR_EOP;

	// Lock FLASH->CR again
	FLASH->CR |= FLASH_CR_LOCK;

	__enable_irq();

	return error;
}

__attribute__((section(".RamFunc"), noinline, optimize("O0")))
uint16_t ProgramFlash(volatile uint32_t *address, uint32_t *data, uint16_t length)
{
	if (length == 0 || (length & 1) || length > 256)
		return 10;

	if (!UnlockFlash())
		return 1;

	__disable_irq();

	while (FLASH->SR & FLASH_SR_BSY1);

	// Clear error status
	FLASH->SR |= FLASH_SR_ERRORS | FLASH_SR_EOP;

	// Enter flash programming mode
	FLASH->CR |= FLASH_CR_PG;

	uint32_t error = 0;
	for (int i = 0; i < length/2; i++)
	{
		address[i*2+0] = data[i*2+0];
		__ISB();
		address[i*2+1] = data[i*2+1];

		while (FLASH->SR & FLASH_SR_BSY1);

		// Clear EOP and check errors
		bool EOP = FLASH->SR & FLASH_SR_EOP;
		FLASH->SR |= FLASH_SR_EOP;
		error = FLASH->SR & FLASH_SR_ERRORS;
		if (!EOP || error) break;
	}

	// Clear any errors
	FLASH->SR |= FLASH_SR_ERRORS;

	// Exist flash programming mode
	FLASH->CR &= ~FLASH_CR_PG;

	// Lock FLASH->CR again
	FLASH->CR = FLASH_CR_LOCK;

	__enable_irq();

	return error;
}

__attribute__((section(".RamFunc"), noinline, optimize("O0")))
uint16_t EraseAndProgramFlash(volatile uint32_t *address, uint32_t *data, uint16_t length)
{
	if (length == 0 || (length & 1) || length > 256)
		return 10;

	if (((uint32_t)address) & 0b111)
		return 10;

	uint16_t page = ((uint32_t)address - FLASH_BASE) / FLASH_PAGE_SIZE;
	uint16_t pageEnd = ((uint32_t)(address+length-1) - FLASH_BASE) / FLASH_PAGE_SIZE;
	if (page != pageEnd)
		return 10;

	LL_WWDG_SetPrescaler(WWDG, LL_WWDG_PRESCALER_8);
	LL_WWDG_SetCounter(WWDG, WWDG_TIMEOUT);

	__disable_irq();

	if (!UnlockFlash())
	{
		__enable_irq();

		LL_WWDG_SetPrescaler(WWDG, LL_WWDG_PRESCALER_1);
		return 1;
	}

	while (FLASH->SR & FLASH_SR_BSY1)
		LL_WWDG_SetCounter(WWDG, WWDG_TIMEOUT);

	// Clear error status
	FLASH->SR |= FLASH_SR_ERRORS | FLASH_SR_EOP;

	// Erase page in flash
	FLASH->CR &= ~FLASH_CR_PNB_Msk;
	FLASH->CR |= FLASH_CR_PER | ((page << FLASH_CR_PNB_Pos) & FLASH_CR_PNB_Msk);
	FLASH->CR |= FLASH_CR_STRT;

	// Wait for operation to finish
	while (FLASH->SR & FLASH_SR_BSY1)
		LL_WWDG_SetCounter(WWDG, WWDG_TIMEOUT);

	// Clear flash erase flag
	FLASH->CR &= ~(FLASH_CR_PER | FLASH_CR_PNB_Msk);

	// Read and clear error status
	uint16_t error = FLASH->SR & FLASH_SR_ERRORS;
	FLASH->SR |= FLASH_SR_ERRORS | FLASH_SR_EOP;

	if (error)
	{
		// Lock FLASH->CR again
		FLASH->CR |= FLASH_CR_LOCK;

		__enable_irq();

		LL_WWDG_SetPrescaler(WWDG, LL_WWDG_PRESCALER_1);
		return error;
	}

	// Enter flash programming mode
	FLASH->CR |= FLASH_CR_PG;

	for (int i = 0; i < length/2; i++)
	{
		LL_WWDG_SetCounter(WWDG, WWDG_TIMEOUT);

		address[i*2+0] = data[i*2+0];
		__ISB();
		address[i*2+1] = data[i*2+1];

		while (FLASH->SR & FLASH_SR_BSY1)
			LL_WWDG_SetCounter(WWDG, WWDG_TIMEOUT);

		// Clear EOP and check errors
		bool EOP = FLASH->SR & FLASH_SR_EOP;
		FLASH->SR |= FLASH_SR_EOP;
		error = FLASH->SR & FLASH_SR_ERRORS;
		if (!EOP || error) break;
	}

	LL_WWDG_SetCounter(WWDG, WWDG_TIMEOUT);

	// Clear any errors
	FLASH->SR |= FLASH_SR_ERRORS;

	// Exit flash programming mode
	FLASH->CR &= ~FLASH_CR_PG;

	// Lock FLASH->CR again
	FLASH->CR = FLASH_CR_LOCK;

	LL_WWDG_SetCounter(WWDG, WWDG_TIMEOUT);

	__enable_irq();

	LL_WWDG_SetPrescaler(WWDG, LL_WWDG_PRESCALER_1);
	return error;
}

void SwitchToBootloader()
{
	struct boot_vectable_ {
		uint32_t Initial_SP;
		void (*Reset_Handler)(void);
	};
#if defined(STM32G0)
	const struct boot_vectable_* BOOTVTAB = (struct boot_vectable_ *)0x1FFF0000;
#else
	#error "Lookup AN2606 for other MCU addresses"
#endif

	// No need for cleanup, only calling this right after startup
	// Proper cleanup is difficult, might look like it works but then e.g. fail to flash

	// Set the MSP
	__set_MSP(BOOTVTAB->Initial_SP);

	// Jump to bootloader
	BOOTVTAB->Reset_Handler();
}


void EnableWatchdog()
{
	// Setup WWDG to freeze during debugging
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_DBGMCU);
	LL_DBGMCU_APB1_GRP1_FreezePeriph(LL_DBGMCU_APB1_GRP1_WWDG_STOP);

	// Enable Watchdog clock
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_WWDG);

	// Setup watchdog to desired interval
	LL_WWDG_SetPrescaler(WWDG, LL_WWDG_PRESCALER_1);
	LL_WWDG_SetWindow(WWDG, 0x7F);
	LL_WWDG_SetCounter(WWDG, WWDG_TIMEOUT);
	// Should be 4096us currently with full WWDG_TIMEOUT at 0x7F
	const uint8_t prescaler = 1; // LL_WWDG_PRESCALER_1
	const uint32_t usTillReset = (WWDG_TIMEOUT - 0x3F) * 4096 * prescaler / SYSCLKFRQ;

	// Prepare watchdog interrupt (mostly used for debug, does not prevent the reset)
	NVIC_SetPriority(WWDG_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(WWDG_IRQn);
	LL_WWDG_EnableIT_EWKUP(WWDG);

	// Enable
	LL_WWDG_Enable(WWDG);
}

void SafeDelayMS(uint16_t ms)
{
	TimePoint tgt = GetTimePoint();
	tgt += ms * TICKS_PER_MS;
	while (GetTimePoint() < tgt)
	{ // In addition to waiting, ensure the watchdog doesn't trigger, since we're expected to wait
		LL_WWDG_SetCounter(WWDG, WWDG_TIMEOUT);
	}
}

/**
 * General IRQ Handlers
 */

void TIM1_BRK_UP_TRG_COM_IRQHandler(void) __IRQ;
void TIM1_BRK_UP_TRG_COM_IRQHandler()
{
	usCounter += TIM1_ARR;
	TIM1->SR = 0;
}

void WWDG_IRQHandler(void) __IRQ;
void WWDG_IRQHandler()
{
	BREAK();
	// Could prevent a reset here, but we're relying on it to recover
	//LL_WWDG_SetCounter(WWDG, WWDG_TIMEOUT);
	LL_WWDG_ClearFlag_EWKUP(WWDG);
}

// Handle Hard Fault
void HardFault_Handler(void) __IRQ;
void HardFault_Handler()
{
	BREAK();
}

/** This function handles Non maskable interrupt. */
void NMI_Handler(void) __IRQ;
void NMI_Handler(void){}

/** This function handles System service call via SWI instruction. */
void SVC_Handler(void) __IRQ;
void SVC_Handler(void){}

/** This function handles Pendable request for system service. */
void PendSV_Handler(void) __IRQ;
void PendSV_Handler(void){}