/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#ifndef __COMPAT_H
#define __COMPAT_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

// Definitions of GPIO Pins / DMA Channels and access so that some descriptive structures can be shared

typedef struct
{ // All chips use same/similar layout, but annoyingly different definitions
	volatile uint32_t CONTROL;
	volatile uint32_t COUNTER;
	volatile uint32_t PER_ADDR;
	volatile uint32_t MEM_ADDR;
} DMA_Channel_Unified;

#if defined(CH32V307)

#define DMA_CHANNEL_1		1
#define DMA_CHANNEL_2		2
#define DMA_CHANNEL_3		3
#define DMA_CHANNEL_4		4
#define DMA_CHANNEL_5		5
#define DMA_CHANNEL_6		6
#define DMA_CHANNEL_7		7
#define DMA_CHANNEL_8		8
#define DMA_CHANNEL_9		9
#define DMA_CHANNEL_10		10
#define DMA_CHANNEL_11		11

// Used only in custom access for CH32V
#define GPIO_PIN_0			0x00000001U
#define GPIO_PIN_1			0x00000002U
#define GPIO_PIN_2			0x00000004U
#define GPIO_PIN_3			0x00000008U
#define GPIO_PIN_4			0x00000010U
#define GPIO_PIN_5			0x00000020U
#define GPIO_PIN_6			0x00000040U
#define GPIO_PIN_7			0x00000080U
#define GPIO_PIN_8			0x00000100U
#define GPIO_PIN_9			0x00000200U
#define GPIO_PIN_10			0x00000400U
#define GPIO_PIN_11			0x00000800U
#define GPIO_PIN_12			0x00001000U
#define GPIO_PIN_13			0x00002000U
#define GPIO_PIN_14			0x00004000U
#define GPIO_PIN_15			0x00008000U

// Same as LL_config for STM32, used only in custom access for CH32V
#define EXTI_LINE_0			0x00000001U
#define EXTI_LINE_1			0x00000002U
#define EXTI_LINE_2			0x00000004U
#define EXTI_LINE_3			0x00000008U
#define EXTI_LINE_4			0x00000010U
#define EXTI_LINE_5			0x00000020U
#define EXTI_LINE_6			0x00000040U
#define EXTI_LINE_7			0x00000080U
#define EXTI_LINE_8			0x00000100U
#define EXTI_LINE_9			0x00000200U
#define EXTI_LINE_10		0x00000400U
#define EXTI_LINE_11		0x00000800U
#define EXTI_LINE_12		0x00001000U
#define EXTI_LINE_13		0x00002000U
#define EXTI_LINE_14		0x00004000U
#define EXTI_LINE_15		0x00008000U

#ifdef __CH32V30x_DMA_H

// Same principle as LL_ functions, but expanded to further channels, as they start having different offsets 
static const uint8_t CHANNEL_OFFSET_TAB[] =
{
	(uint8_t)(DMA2_Channel1_BASE - DMA2_BASE),
	(uint8_t)(DMA2_Channel2_BASE - DMA2_BASE),
	(uint8_t)(DMA2_Channel3_BASE - DMA2_BASE),
	(uint8_t)(DMA2_Channel4_BASE - DMA2_BASE),
	(uint8_t)(DMA2_Channel5_BASE - DMA2_BASE),
	(uint8_t)(DMA2_Channel6_BASE - DMA2_BASE),
	(uint8_t)(DMA2_Channel7_BASE - DMA2_BASE),
	(uint8_t)(DMA2_Channel8_BASE - DMA2_BASE),
	(uint8_t)(DMA2_Channel9_BASE - DMA2_BASE),
	(uint8_t)(DMA2_Channel10_BASE - DMA2_BASE),
	(uint8_t)(DMA2_Channel11_BASE - DMA2_BASE)
};

#define DMA_CONTROL_ENABLE 0b1

#define DMA_CH(DMAx, CH) ((DMA_Channel_Unified *)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[CH-DMA_CHANNEL_1]))

#endif

#ifdef __CH32V30x_GPIO_H

// Input have 00 in lower 2 bits
#define GPIO_ANALOG_IN 0b0000
#define GPIO_FLOATING_IN 0b0100
#define GPIO_PU_PD_IN 0b1000 // Pull-Up + Pull-Down selectable
// Output need to be combined with a GPIO_Speed_ for the lower 2 bits
#define GPIO_PP_OUT 0b0000
#define GPIO_OD_OUT 0b0100
// Altnerate Function is for peripheral instead of general IO
#define GPIO_AF_PP_OUT 0b1000
#define GPIO_AF_OD_OUT 0b1100

#define GPIO_CFGR(GPIOx, PIN) *(PIN < GPIO_PIN_8? &GPIOx->CFGLR : &GPIOx->CFGHR)
#define GPIO_CFGR_POS(PIN) (PIN < GPIO_PIN_8? ((__builtin_ffs(PIN)-1)<<2) : ((__builtin_ffs(PIN)-9)<<2))

static inline __attribute__((always_inline)) void GPIO_CFG(GPIO_TypeDef *GPIOx, uint32_t PIN, uint32_t CFG)
{
	GPIO_CFGR(GPIOx, PIN) &= ~(0xF << GPIO_CFGR_POS(PIN));
	GPIO_CFGR(GPIOx, PIN) |= (CFG&0xF) << GPIO_CFGR_POS(PIN);
}

#define GPIO_SET(GPIO, PINS) GPIO->BSHR = (PINS)&0xFFFF
#define GPIO_RESET(GPIO, PINS) GPIO->BCR = (PINS)&0xFFFF
#define GPIO_READ(GPIO, PINS) (GPIO->INDR&(PINS))

#endif

// Every IRQ NEEDs this in this RISC-V core (in contrast to ARM)
// Otherwise it will not disable the corresponding NVIC Handler, and stay active
// That will prevent interrupts of same or lower preemptive priority to never trigger again
#define __IRQ __attribute__((interrupt("machine")))

#else

#if defined(STM32F1xx_LL_GPIO_H) | defined(__STM32F3xx_LL_GPIO_H) | defined(STM32G0xx_LL_GPIO_H)

// Used only in LL_ functions for STM32
#define GPIO_PIN_0			LL_GPIO_PIN_0
#define GPIO_PIN_1			LL_GPIO_PIN_1
#define GPIO_PIN_2			LL_GPIO_PIN_2
#define GPIO_PIN_3			LL_GPIO_PIN_3
#define GPIO_PIN_4			LL_GPIO_PIN_4
#define GPIO_PIN_5			LL_GPIO_PIN_5
#define GPIO_PIN_6			LL_GPIO_PIN_6
#define GPIO_PIN_7			LL_GPIO_PIN_7
#define GPIO_PIN_8			LL_GPIO_PIN_8
#define GPIO_PIN_9			LL_GPIO_PIN_9
#define GPIO_PIN_10			LL_GPIO_PIN_10
#define GPIO_PIN_11			LL_GPIO_PIN_11
#define GPIO_PIN_12			LL_GPIO_PIN_12
#define GPIO_PIN_13			LL_GPIO_PIN_13
#define GPIO_PIN_14			LL_GPIO_PIN_14
#define GPIO_PIN_15			LL_GPIO_PIN_15

#define GPIO_SET(GPIO, PINS) LL_GPIO_SetOutputPin(GPIO, PINS)
#define GPIO_RESET(GPIO, PINS) LL_GPIO_ResetOutputPin(GPIO, PINS)
#define GPIO_READ(GPIO, PINS) (LL_GPIO_ReadInputPort(GPIO)&(PINS))

#endif

#if defined(STM32F1xx_LL_DMA_H) | defined(__STM32F3xx_LL_DMA_H) | defined(STM32G0xx_LL_DMA_H)

// Same as LL_config for STM32, used only in custom access for CH32V
#define DMA_CHANNEL_1		LL_DMA_CHANNEL_1
#define DMA_CHANNEL_2		LL_DMA_CHANNEL_2
#define DMA_CHANNEL_3		LL_DMA_CHANNEL_3
#define DMA_CHANNEL_4		LL_DMA_CHANNEL_4
#define DMA_CHANNEL_5		LL_DMA_CHANNEL_5
#define DMA_CHANNEL_6		LL_DMA_CHANNEL_6
#define DMA_CHANNEL_7		LL_DMA_CHANNEL_7
#define DMA_CHANNEL_8		LL_DMA_CHANNEL_8
#define DMA_CHANNEL_9		LL_DMA_CHANNEL_9
#define DMA_CHANNEL_10		LL_DMA_CHANNEL_10
#define DMA_CHANNEL_11		LL_DMA_CHANNEL_11

#define DMA_CONTROL_ENABLE 0b1	// Same on all of them (DMA_CFGR1_EN, DMA_CCR_EN)

#define DMA_CH(DMAx, CH) ((DMA_Channel_Unified *)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[CH-DMA_CHANNEL_1]))

#endif

#define __IRQ __attribute__((interrupt("IRQ")))

#endif

#if defined(GD32F303R)
// From above headers, can't include because headers clash with STM32 ones (conflicting definitions because of different naming scheme)
//#include "gd32f30x.h"
#define UART4_IRQn						52
#define DMA2_Channel3_IRQn				58
#define DMA2_Channel4_Channel5_IRQn		59
//#include "gd32f30x_rcu.h"
#define LL_APB1_GRP1_PERIPH_UART4		(1<<19)
#define LL_AHB1_GRP1_PERIPH_DMA2		(1<<1)
//#include "gd32f30x_dma.h"
#define DMA2	((DMA_TypeDef *)(DMA1_BASE + 0x0400U))
//#include "gd32f30x_usart.h"
#define UART4	((USART_TypeDef *)(USART3_BASE + 0x0400U))
#endif

#if defined(__CH32V30x_H)

#define TIM_SR(TIM) TIM->INTFR

static inline void StartTimer(TIM_TypeDef *TIM, uint16_t interval)
{
	TIM->ATRLR = interval-1; // Set timer interval
	TIM->SWEVGR |= TIM_UG; // Trigger update event to load interval
	TIM->DMAINTENR = TIM_UIE; // Enable update interrupt
	TIM->CTLR1 |= TIM_CEN; // Enable timer
}

static inline void StopTimer(TIM_TypeDef *TIM)
{
	TIM->CTLR1 &= ~TIM_CEN;
}

#elif defined(STM32F1xx_LL_TIM_H) | defined(__STM32F3xx_LL_TIM_H) | defined(STM32G0xx_LL_TIM_H)

#define TIM_SR(TIM) TIM->SR
#define TIM_UIF TIM_SR_UIF

static inline void StartTimer(TIM_TypeDef *TIM, uint16_t interval)
{
	TIM->ARR = interval-1; // Set timer interval
	TIM->EGR |= TIM_EGR_UG; // Trigger update event to load interval
	TIM->DIER = TIM_DIER_UIE; // Enable update interrupt
	TIM->CR1 |= TIM_CR1_CEN; // Enable timer
}

static inline void StopTimer(TIM_TypeDef *TIM)
{
	TIM->CR1 &= ~TIM_CR1_CEN;
}

#endif


#ifdef __cplusplus
}
#endif

#endif /* __COMPAT_H */
