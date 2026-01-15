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
#include "stm32g0xx_ll_tim.h"
#include "stm32g0xx_ll_dma.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_system.h"
#endif

#include "config_impl.h"
#include "rgbled.h"
#include "util.h"

#include <stdint.h>
#include <string.h>

// Definitions of GPIO Pins / DMA Channels and access so that some descriptive structures (UART_DMA_Setup UART) can be shared
#include "compat.h"

/* Functions */

const uint8_t MAX_TIM_CNT = SYSCLKFRQ*1000/800; // 800Khz with no prescaler for WS2812 LED PWM Frequency
const uint8_t BIT_HIGH = MAX_TIM_CNT * 2 / 3;
const uint8_t BIT_LOW = MAX_TIM_CNT * 1 / 3;

static uint8_t RGBLED_PWM_BUFFER[RGBLED_COUNT * 3 * 8 + 1];

static uint32_t DMA_CH = DMA_CHANNEL_1;

static volatile bool writing;
static volatile bool ready;
const uint16_t resetTimeUS = 60;

static TimePoint readyEarliest;
static TimePoint wakeupEarliest;

static void rgbled_output_LOW()
{
	LL_GPIO_SetPinMode(WS2812_GPIO_X, WS2812_PIN, LL_GPIO_MODE_OUTPUT);
	GPIO_RESET(WS2812_GPIO_X, WS2812_PIN);
}

static void rgbled_output_PWM()
{
	LL_GPIO_SetPinMode(WS2812_GPIO_X, WS2812_PIN, LL_GPIO_MODE_ALTERNATE);
}

void rgbled_init_driver()
{
	// GPIO Clocks are already active

	// DMA Peripheral clock enable
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

	// WS2812 Output
	if (WS2812_PIN < GPIO_PIN_8)
		LL_GPIO_SetAFPin_0_7(WS2812_GPIO_X, WS2812_PIN, LL_GPIO_AF_2);
	else
		LL_GPIO_SetAFPin_8_15(WS2812_GPIO_X, WS2812_PIN, LL_GPIO_AF_2);
	LL_GPIO_SetPinSpeed(WS2812_GPIO_X, WS2812_PIN, LL_GPIO_SPEED_FREQ_VERY_HIGH);
	LL_GPIO_SetPinOutputType(WS2812_GPIO_X, WS2812_PIN, LL_GPIO_OUTPUT_PUSHPULL);
	rgbled_output_LOW();

	// -- PWM TIMER --

	NVIC_SetPriority(TIM16_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));
	NVIC_EnableIRQ(TIM16_IRQn);

	// Initialise TIM16
	TIM16->CR1 &= ~TIM_CR1_CEN; // Disable Counter
	RCC->APBRSTR2 |= RCC_APBRSTR2_TIM16RST; // Reset TIM16
	RCC->APBRSTR2 &= ~RCC_APBRSTR2_TIM16RST;
	RCC->APBENR2 |= RCC_APBENR2_TIM16EN; // Enable internal clock for TIM16

	// Setup TIM16 for WS2812
	TIM16->CCMR1 = 
		(TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1)	// Channel active == counter < channel ccr
		| TIM_CCMR1_OC1PE						// OC1 Preload enable
		| TIM_CCMR1_OC1FE;						// OC1 Fast enable
	TIM16->CCER = TIM_CCER_CC1E;				// Enable Active High output
	TIM16->BDTR = TIM_BDTR_MOE;					// Main Output Enable
	TIM16->CR1 = TIM_CR1_URS;	// Only generate update on overflow;
	TIM16->DIER = TIM_DIER_UDE;		// Enable Update DMA Request

	// -- DMA CCR (Compare Register) Reload --
	// NOTE: We're only driving one chain of 4 LEDs with one CC channel, so the DMA Burst feature (using DCR and DMAR) is not used

	NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	// Setup DMAMUX found on STM32G0 series
#if defined(STM32G0)
	LL_DMA_SetPeriphRequest(DMA1, DMA_CH, LL_DMAMUX_REQ_TIM16_UP);
#endif

	// Setup respective DMA Channel for Updating TIM16 CCR
	LL_DMA_SetDataTransferDirection(DMA1, DMA_CH, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetChannelPriorityLevel(DMA1, DMA_CH, LL_DMA_PRIORITY_HIGH);
	LL_DMA_SetMode(DMA1, DMA_CH, LL_DMA_MODE_NORMAL);
	LL_DMA_SetPeriphIncMode(DMA1, DMA_CH, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA1, DMA_CH, LL_DMA_MEMORY_INCREMENT);
	// Periphery Size always has to be Halfword (16bit), found no tricks around it, even though we're only writing 8bit of it
	LL_DMA_SetPeriphSize(DMA1, DMA_CH, LL_DMA_PDATAALIGN_HALFWORD);
	LL_DMA_SetMemorySize(DMA1, DMA_CH, LL_DMA_MDATAALIGN_BYTE);
	LL_DMA_SetPeriphAddress(DMA1, DMA_CH, ((uint32_t)&TIM16->CCR1));

	// Enable TX Interrupt (TC)
	LL_DMA_EnableIT_TC(DMA1, DMA_CH);

	ready = true;
	writing = false;
}

bool rgbled_set_and_delay(uint8_t rgb[RGBLED_COUNT*3], TimePoint earliest)
{
	if (!ready) return false;

	ready = false;
	writing = true;

	// Set the PWM data for all RGB LEDs
	for (int l = 0; l < RGBLED_COUNT; l++)
	{
		for (int c = 0; c < 3; c++)
		{
			const uint8_t map[] = { 1, 0, 2 }; // Map GRB to RGB
			uint8_t value = rgb[l*3 + map[c]];
			for (int b = 0; b < 8; b++)
				RGBLED_PWM_BUFFER[l*24+c*8+b] = (value >> (7-b)) & 1? BIT_HIGH : BIT_LOW;
		}
	}
	RGBLED_PWM_BUFFER[RGBLED_COUNT * 3 * 8] = 0; // Trailing byte to force PWM signal low (and be able to stop timer on TC interrupt)

	// Trigger Update to preload CCR with 0 (for 0 on PWM)
	TIM16->CCR1 = 0;
	TIM16->EGR |= TIM_EGR_UG;

	// Prepare DMA to reload CCR /this will also trigger the first DMA to set CCR for next update)
	LL_DMA_SetMemoryAddress(DMA1, DMA_CH, (uint32_t)(intptr_t)&RGBLED_PWM_BUFFER);
	LL_DMA_SetDataLength(DMA1, DMA_CH, RGBLED_COUNT * 3 * 8 + 1);
	LL_DMA_EnableChannel(DMA1, DMA_CH);

	// Enable the timer PWM - will write 0 until first real update, when it reloads DMAd CCR value
	TIM16->CNT = 0;						// Reset Counter
	TIM16->SR = 0;						// Reset Status Register
	TIM16->PSC = 0;						// Disable prescaler (SYSCLKFRQ Mhz)
	TIM16->ARR = MAX_TIM_CNT-1;			// Set Auto-Reload to trigger at 800Khz / 1.25us interval
	TIM16->CR1 |= TIM_CR1_CEN;

	// Set output to PWM from TIM16
	rgbled_output_PWM();

	// Tell rgbled_stop to set earliest wakeup/ready time
	wakeupEarliest = earliest;

	return true;
}

bool rgbled_set(uint8_t rgb[RGBLED_COUNT*3])
{
	return rgbled_set_and_delay(rgb, GetTimePoint());
}

static void rgbled_set_ready_timer(TimeSpan time)
{
	uint16_t prescaler = 1;
	unsigned int waitPeriods = time * SYSCLKFRQ / MAX_TIM_CNT / TICKS_PER_US;
	while (waitPeriods >= (1 << 16))
	{ // Need prescaler
		prescaler *= 10;
		waitPeriods /= 10;
	}

	// Configure timer for reset pulse after which we get a callback
	TIM16->CNT = 0;						// Reset Counter
	TIM16->SR = 0;						// Reset Status Register
	TIM16->PSC = prescaler-1;			// Set prescaler higher if wait time demands
	TIM16->ARR = waitPeriods-1;			// Set Auto-Reload to trigger after 40 periods (50us)
	TIM16->DIER |= TIM_DIER_UIE;		// Generate one update interrupt after reset pulse
	TIM16->CR1 |= TIM_CR1_CEN;
}

static void rgbled_abort_ready_timer()
{
	// Reset timer
	TIM16->CR1 &= ~TIM_CR1_CEN;		// Disable Counter
	TIM16->DIER &= ~TIM_DIER_UIE;	// Remove update interrupt enable
}

void rgbled_stop()
{
	// Just sent trailing byte with DMA, so TIM16 should have CCR == 0 - we can disable and set output

	// Disable output, dma and timer
	rgbled_output_LOW();
	LL_DMA_DisableChannel(DMA1, DMA_CH);
	TIM16->CR1 &= ~TIM_CR1_CEN;

	// Update state - ready should already have been false
	writing = false;
	ready = false;

	// Set ready timer (potentially with additional delay if no wakeup is required for a while)
	TimePoint now = GetTimePoint();
	readyEarliest = now + 50 * TICKS_PER_US; // Latch time of 50us
	if (readyEarliest < wakeupEarliest)
		rgbled_set_ready_timer(wakeupEarliest - now);
	else
		rgbled_set_ready_timer(readyEarliest - now);
}

bool rgbled_ready()
{
	return ready;
}

bool rgbled_abort_wakeup_waiting()
{
	wakeupEarliest = GetTimePoint();
	if (writing) return false;
	if (ready) return true;
	// Reevaluate ready - discard wakeup, just focus on reset period
	TimePoint now = GetTimePoint();
	ready = now > readyEarliest;
	if (!ready) // Might have aborted wakeup timer, but still not ready yet
		rgbled_set_ready_timer(readyEarliest - now);
	return ready;
}

volatile bool locked = false;

void rgbled_lock()
{
	USE_LOCKS();
	LOCK();
	// If locked, we are interrupting a lower priority IRQ or the main thread
	// That indicates this calling IRQs has not been accounted for in below IRQ
	// TODO: Rework
	if (locked) BREAK();
	// Acts as an assertion (will eventually trigger watchdog)
	while (locked);
	locked = true;
	NVIC_DisableIRQ(TIM16_IRQn);
	NVIC_DisableIRQ(DMA1_Channel1_IRQn);
	NVIC_DisableIRQ(USART1_IRQn);
	NVIC_DisableIRQ(DMA1_Channel2_3_IRQn);
	NVIC_DisableIRQ(I2C1_IRQn);
	UNLOCK();
}

void rgbled_unlock()
{
	USE_LOCKS();
	LOCK();
	locked = false;
	NVIC_EnableIRQ(TIM16_IRQn);
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
	NVIC_EnableIRQ(I2C1_IRQn);
	UNLOCK();
}

/** Interrupt handlers */

// User function
void rgbled_ready_callback();

void TIM16_IRQHandler() __IRQ;
void TIM16_IRQHandler()
{
	if (TIM16->SR & TIM_SR_UIF)
	{ // Update - only enabled for reset pulse
		rgbled_abort_ready_timer();

		// Signal rgbled is ready
		ready = true;
		rgbled_ready_callback();
	}
	TIM16->SR = 0;
}

void DMA1_Channel1_IRQHandler() __IRQ;
void DMA1_Channel1_IRQHandler()
{
	if (LL_DMA_IsActiveFlag_TE1(DMA1))
	{ // PWM TX FAIL
		LL_DMA_ClearFlag_TE1(DMA1);
	}
	if (LL_DMA_IsActiveFlag_TC1(DMA1))
	{ // PWM TX Full
		LL_DMA_ClearFlag_TC1(DMA1);
		rgbled_stop();
	}
}