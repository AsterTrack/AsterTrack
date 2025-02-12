/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "ch32v30x.h"
#include "ch32v30x_rcc.h"
#include "ch32v30x_gpio.h"
#include "compat.h"

void SystemInit(void)
{
	// Set to 72Mhz
	// Anything above 100Mhz and Flash will have problems writing

	RCC->CTLR |= (uint32_t)0x00000001;

#ifdef CH32V30x_D8C
	RCC->CFGR0 &= (uint32_t)0xF8FF0000;
#else
	RCC->CFGR0 &= (uint32_t)0xF0FF0000;
#endif

	RCC->CTLR &= (uint32_t)0xFEF6FFFF;
	RCC->CTLR &= (uint32_t)0xFFFBFFFF;
	RCC->CFGR0 &= (uint32_t)0xFF80FFFF;

#ifdef CH32V30x_D8C
	RCC->CTLR &= (uint32_t)0xEBFFFFFF;
	RCC->INTR = 0x00FF0000;
	RCC->CFGR2 = 0x00000000;
#else
	RCC->INTR = 0x009F0000;   
#endif

	__IO uint32_t StartUpCounter = 0, HSEStatus = 0;

	RCC->CTLR |= ((uint32_t)RCC_HSEON);

	/* Wait till HSE is ready and if Time out is reached exit */
	do
	{
		HSEStatus = RCC->CTLR & RCC_HSERDY;
		StartUpCounter++;  
	} while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

	if ((RCC->CTLR & RCC_HSERDY) != RESET)
	{
		/* HCLK = SYSCLK */
		RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV1; 
		/* PCLK2 = HCLK */
		RCC->CFGR0 |= (uint32_t)RCC_PPRE2_DIV1; 
		/* PCLK1 = HCLK */
		RCC->CFGR0 |= (uint32_t)RCC_PPRE1_DIV2;

		/* PLL configuration: PLLCLK = HSE * 9 = 72 MHz */
		RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_PLLSRC | RCC_PLLXTPRE | RCC_PLLMULL));

	#ifdef CH32V30x_D8
		RCC->CFGR0 |= (uint32_t)(RCC_PLLSRC_HSE | RCC_PLLXTPRE_HSE | RCC_PLLMULL9);
	#else
		RCC->CFGR0 |= (uint32_t)(RCC_PLLSRC_HSE | RCC_PLLXTPRE_HSE | RCC_PLLMULL9_EXTEN);
	#endif

		/* Enable PLL */
		RCC->CTLR |= RCC_PLLON;
		/* Wait till PLL is ready */
		while((RCC->CTLR & RCC_PLLRDY) == 0);
		/* Select PLL as system clock source */
		RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_SW));
		RCC->CFGR0 |= (uint32_t)RCC_SW_PLL;    
		/* Wait till PLL is used as system clock source */
		while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x08);
	}
}

void SetRomRamSplit()
{
	uint8_t desiredSRAMModeCode = 0b00; // ROM/Flash 192KB, RAM 128KB

	// Test SRAM Mode Code if it's already in the desired state
	uint8_t SRAMModeCode = (OB->USER >> 6) & 0b11;
	if (SRAMModeCode == desiredSRAMModeCode)
	{ // Already in desired state, leave
		while(1){}
		//desiredSRAMModeCode = (desiredSRAMModeCode+1)&0b11;
	}
	
	//for (int i = 0; i < 1000; i++);

	// Unlock Flash
	FLASH->KEYR = 0x45670123;
	FLASH->KEYR = 0xCDEF89AB;
	if (FLASH->CTLR & FLASH_CTLR_LOCK)
		while(1){} // Failed to unlock flash
	while (FLASH->STATR & FLASH_STATR_BSY); // Busy, cannot access flash

	// Test code again, different way to access it
	SRAMModeCode = (FLASH->OBR >> 8) & 0b11;
	if (SRAMModeCode == desiredSRAMModeCode)
	{ // Already in desired state, unlock and leave
		FLASH->CTLR |= FLASH_CTLR_LOCK;
		while(1){}
	}
	
	//for (int i = 0; i < 1000; i++);

	// Unlock Option Bytes
	FLASH->OBKEYR = 0x45670123;
	FLASH->OBKEYR = 0xCDEF89AB;
	if ((FLASH->CTLR & FLASH_CTLR_OPTWRE) != FLASH_CTLR_OPTWRE)
		while(1){} // Failed to unlock

	//for (int i = 0; i < 1000; i++);

	// Read option byte and decide what to write
	uint8_t USER = OB->USER&0xFF;
	uint8_t USER_MOD = (USER & 0b00111111) | (desiredSRAMModeCode << 6);
	uint16_t USER_REG = (~USER_MOD << 8) | USER_MOD;

	// Start Option Byte Programming
	FLASH->CTLR |= FLASH_CTLR_OPTPG; // Option Byte Programming
	FLASH->CTLR |= FLASH_CTLR_STRT; // Start process

	//for (int i = 0; i < 100; i++);

	// Write USER Option Byte
	OB->USER = USER_REG;
	while (FLASH->STATR & FLASH_STATR_BSY);
	FLASH->CTLR |= FLASH_STATR_EOP; // Clear EOP bit

	//for (int i = 0; i < 100; i++);

	// Leave and unlock Option Byte Programming
	FLASH->CTLR &= ~FLASH_CTLR_OPTPG; // Leave Option Byte Programming
	FLASH->CTLR |= FLASH_CTLR_OPTWRE; // Release Option Byte Lock
	FLASH->CTLR |= FLASH_CTLR_LOCK; // Release Flash Lock

	//for (int i = 0; i < 1000; i++);

	// Verify code got changed as desired
	SRAMModeCode = (OB->USER >> 6) & 0b11;
	if (SRAMModeCode != desiredSRAMModeCode)
		while(1){} // Failed to write
	
	//for (int i = 0; i < 1000; i++);

	// Now perform System Reset to apply values (normal power-cycle is not enough)
	// Either of them should do it
	PFIC->CFGR = NVIC_KEY3 | 0x0080;
	PFIC->SCTLR = 0x8000;

	// TODO: Something here doesn't work yet, it doesn't properly apply
	// The only way I can get it to apply is to enter debug mode, and restart
		// This will jump into the "Already in desired state, leave" check
		// Then simply disconnect and it will be applied
		// However, this is also not 100% reliable
		// -- Is this still the case? Unsure

	while(1){}
}

int main(void)
{
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


	// -- Configure Chip --

	SetRomRamSplit();
}

/**
 * Reset/Flash button pressed
 */
void EXTI1_IRQHandler(void) __IRQ;
void EXTI1_IRQHandler()
{
	if (EXTI->INTFR & EXTI_LINE_1)
	{ // Interrupt pending

		// System Reset
		PFIC->CFGR = NVIC_KEY3 | 0x0080;

		// If BOOT0 is still pressed, it will reboot into bootloader
	}

	// Reset IRQ flag
	EXTI->INTFR = EXTI_LINE_1;
}

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}