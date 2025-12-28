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

#ifndef __UTIL_H
#define __UTIL_H

#ifdef __cplusplus
extern "C"
{
#endif

#if defined(CH32V307)
#include "ch32v30x.h"
#elif defined(STM32)
#include "stm32.h"
#endif

#include "comm/packet.h"

#include <stdlib.h> //#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

static const uint8_t hex[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };

#if defined(CH32V307)

#define SYSCLKFRQ 144

#define PCLK2 SYSCLKFRQ*1000000
// Same max clock as system frequency
#define PCLK1 SYSCLKFRQ*1000000

#elif defined(STM32)

#define SYSCLKFRQ 64

#define PCLK SYSCLKFRQ*1000000

#endif

/* Mutex/Critical Zones */

#define USE_LOCKS() bool ginten

#if defined(CH32V307)

static inline uint32_t __get_GINTENR()
{
  uint32_t result;
  __asm volatile("csrr %0,""0x800" : "=r"(result));
  return result;
}

static inline uint32_t __get_GINTEN()
{
	return __get_GINTENR() != 0x6000;
}

#elif defined(STM32)

#define __get_GINTEN __get_PRIMASK

#endif

#define LOCK() { \
	ginten = __get_GINTEN(); \
	__disable_irq(); \
}

#define UNLOCK() { \
	if (ginten) \
		__enable_irq(); \
}

#define ATOMIC(EXP) { \
	LOCK() \
	{ \
		EXP \
	} \
	UNLOCK() \
}

#define ATOMIC_SINGLE(EXP) { \
	__disable_irq(); \
	{ \
		EXP \
	} \
	__enable_irq(); \
}

#define BREAK() __asm("EBREAK;")


/* Debug */

// NO need for atomics in the current configurations, as long as:
// TIM2, TIM3 interrupts do not use DEBUG (they preempt other code with )debugs
// main only uses debugs in PacketHubZone (no USB&UART interrupts)
// USB&UART interrupts may use debugs freely, as they don't preempt each other

#include <string.h>

#if defined(ENABLE_LOG)

#if defined(LOG_USE_SDI)
void SDI_Enable();
bool SDI_ProbeAttached();
bool SDI_Busy();
bool SDI_Write(uint8_t *buf, uint_fast16_t size);
#endif

#define DEBUG_BUFFER_SIZE (1<<14)
extern volatile uint_fast16_t debugHead, debugTail, debugSending;
extern uint8_t *debugBuffer; // [DEBUG_BUFFER_SIZE];

__attribute__((unused))
static void LOG_BUF(const void *buffer, uint_fast16_t size)
{
	int pos = debugHead;
	int space = debugTail-pos;
	if (space <= 0) space += DEBUG_BUFFER_SIZE;
	if (size < space)
	{
		int end = pos + size;
		int spill = end - DEBUG_BUFFER_SIZE;
		if (spill <= 0)
		{
			debugHead = spill == 0? 0 : end;
			memcpy(&debugBuffer[pos], buffer, size);
		}
		else
		{
			debugHead = spill;
			uint16_t sz1 = DEBUG_BUFFER_SIZE - pos;
			memcpy(&debugBuffer[pos], buffer, sz1);
			memcpy(&debugBuffer[0], (buffer)+sz1, spill);
		}
	}
	else if (space > 1)
	{
		int end = pos + space - 1;
		int spill = end - DEBUG_BUFFER_SIZE;
		if (spill <= 0)
		{
			debugHead = spill == 0? 0 : end;
			memset(&debugBuffer[pos], '*', space-1);
		}
		else
		{
			debugHead = spill;
			uint16_t sz1 = DEBUG_BUFFER_SIZE - pos;
			memset(&debugBuffer[pos], '*', sz1);
			memset(&debugBuffer[0], '*', spill);
		}
	}
}

#define LOG_CHARR(...) \
do { \
	const char arr[] = { __VA_ARGS__ }; \
	LOG_BUF(arr, sizeof(arr)); \
} while(false)

#define LOG_STR(STR) \
do { \
	const char arr[] = STR; \
	LOG_BUF(arr, sizeof(arr)-1); \
} while(false)
#define LOG_HEX(BUF, SZ) \
do { \
	char arr[(SZ)*2]; \
	for (int i = 0; i < (SZ); i++) \
	{ \
		arr[2*i+0] = hex[((uint8_t*)(BUF))[i] >> 4]; \
		arr[2*i+1] = hex[((uint8_t*)(BUF))[i] & 0xF]; \
	} \
	LOG_BUF(arr, sizeof(arr)); \
} while(false)
#define UI32_TO_HEX_ARR(val) UI16_TO_HEX_ARR((val) >> 16), UI16_TO_HEX_ARR((val) & 0xFFFF)
#define UI16_TO_HEX_ARR(val) UI8_TO_HEX_ARR((val) >> 8), UI8_TO_HEX_ARR((val) & 0xFF)
#define UI8_TO_HEX_ARR(val) hex[((val) >> 4)&0xF], hex[((val) >> 0)&0xF]
#define UINT9_TO_CHARR(val) '0'+(val)%10
#define UINT99_TO_CHARR(val) '0'+((val)%100)/10, UINT9_TO_CHARR(val)
#define UINT999_TO_CHARR(val) '0'+((val)%1000)/100, UINT99_TO_CHARR(val)
#define UINT9999_TO_CHARR(val) '0'+((val)%10000)/1000, UINT999_TO_CHARR(val)
#define UINT99999_TO_CHARR(val) '0'+((val)%100000)/10000, UINT9999_TO_CHARR(val)
#define UINT999999_TO_CHARR(val) '0'+((val)%1000000)/100000, UINT99999_TO_CHARR(val)
#define UINT9999999_TO_CHARR(val) '0'+((val)%10000000)/1000000, UINT999999_TO_CHARR(val)
#define UINT99999999_TO_CHARR(val) '0'+((val)%100000000)/10000000, UINT9999999_TO_CHARR(val)
#define INT999999999_TO_CHARR(val) val < 0? '~' : '0'+((val)%1000000000)/100000000, UINT99999999_TO_CHARR(abs(val))
#define INT99999999_TO_CHARR(val) val < 0? '~' : '0'+((val)%100000000)/10000000, UINT9999999_TO_CHARR(abs(val))
#define INT9999999_TO_CHARR(val) val < 0? '~' : '0'+((val)%10000000)/1000000, UINT999999_TO_CHARR(abs(val))
#define INT999999_TO_CHARR(val) val < 0? '~' : '0'+((val)%1000000)/100000, UINT99999_TO_CHARR(abs(val))
#define INT99999_TO_CHARR(val) val < 0? '~' : '0'+((val)%100000)/10000, UINT9999_TO_CHARR(abs(val))
#define INT9999_TO_CHARR(val) val < 0? '~' : '0'+((val)%10000)/1000, UINT999_TO_CHARR(abs(val))
#define INT999_TO_CHARR(val) val < 0? '~' : '0'+((val)%1000)/100, UINT99_TO_CHARR(abs(val))	
#define INT99_TO_CHARR(val) val < 0? '~' : '0'+((val)%100)/10, UINT9_TO_CHARR(abs(val))
#define INT9_TO_CHARR(val) val < 0? '~' : '0'+(val)%10

#else

#define LOG_CHARR(...) do {} while(false)
#define LOG_STR(...) do {} while(false)
#define LOG_HEX(...) do {} while(false)

#endif// Any Log

#if DEBUG_LOG
#define DEBUG_CHARR LOG_CHARR
#define DEBUG_STR LOG_STR
#define DEBUG_HEX LOG_HEX
#else
#define DEBUG_CHARR(...) do {} while(false)
#define DEBUG_STR(...) do {} while(false)
#define DEBUG_HEX(...) do {} while(false)
#endif // DEBUG_LOG

#if INFO_LOG
#define INFO_CHARR LOG_CHARR
#define INFO_STR LOG_STR
#define INFO_HEX LOG_HEX
#else
#define INFO_CHARR(...) do {} while(false)
#define INFO_STR(...) do {} while(false)
#define INFO_HEX(...) do {} while(false)
#endif // INFO_LOG

#if WARN_LOG
#define WARN_CHARR LOG_CHARR
#define WARN_STR LOG_STR
#define WARN_HEX LOG_HEX
#else
#define WARN_CHARR(...) do {} while(false)
#define WARN_STR(...) do {} while(false)
#define WARN_HEX(...) do {} while(false)
#endif // WARN_LOG

#if ERR_LOG
#define ERR_CHARR LOG_CHARR
#define ERR_STR LOG_STR
#define ERR_HEX LOG_HEX
#else
#define ERR_CHARR(...) do {} while(false)
#define ERR_STR(...) do {} while(false)
#define ERR_HEX(...) do {} while(false)
#endif // ERR_LOG

#if KERR_LOG
#define KERR_CHARR LOG_CHARR
#define KERR_STR LOG_STR
#else
#define KERR_CHARR(...) do {} while(false)
#define KERR_STR(...) do {} while(false)
#endif // KERR_LOG

#if TERR_LOG
#define TERR_STR LOG_STR
#define TERR_CHARR LOG_CHARR
#else
#define TERR_STR(...) do {} while(false)
#define TERR_CHARR(...) do {} while(false)
#endif // TERR_LOG

#if TEMP_LOG
#define TEMP_CHARR LOG_CHARR
#define TEMP_STR LOG_STR
#else
#define TEMP_CHARR(...) do {} while(false)
#define TEMP_STR(...) do {} while(false)
#endif

#if COMM_LOG
#define COMM_CHARR LOG_CHARR
#define COMM_STR LOG_STR
#else
#define COMM_CHARR(...) do {} while(false)
#define COMM_STR(...) do {} while(false)
#endif

#if CMDD_LOG
#define CMDD_CHARR LOG_CHARR
#define CMDD_STR LOG_STR
#else
#define CMDD_CHARR(...) do {} while(false)
#define CMDD_STR(...) do {} while(false)
#endif

#if USBD_LOG
#define USBD_CHARR LOG_CHARR
#define USBD_STR LOG_STR
#else
#define USBD_CHARR(...) do {} while(false)
#define USBD_STR(...) do {} while(false)
#endif

#if USBC_LOG
#define USBC_CHARR LOG_CHARR
#define USBC_STR LOG_STR
#else
#define USBC_CHARR(...) do {} while(false)
#define USBC_STR(...) do {} while(false)
#endif

#if USBP_LOG
#define USBP_CHARR LOG_CHARR
#define USBP_STR LOG_STR
#else
#define USBP_CHARR(...) do {} while(false)
#define USBP_STR(...) do {} while(false)
#endif

#if USBPD_LOG
#define USBPD_BUF LOG_BUF
#define USBPD_CHARR LOG_CHARR
#define USBPD_STR LOG_STR
#else
#define USBPD_BUF(...) do {} while(false)
#define USBPD_CHARR(...) do {} while(false)
#define USBPD_STR(...) do {} while(false)
#endif

#if POW_LOG
#define POW_BUF LOG_BUF
#define POW_CHARR LOG_CHARR
#define POW_STR LOG_STR
#else
#define POW_BUF(...) do {} while(false)
#define POW_CHARR(...) do {} while(false)
#define POW_STR(...) do {} while(false)
#endif

#if ADC_LOG
#define ADC_BUF LOG_BUF
#define ADC_CHARR LOG_CHARR
#define ADC_STR LOG_STR
#else
#define ADC_BUF(...) do {} while(false)
#define ADC_CHARR(...) do {} while(false)
#define ADC_STR(...) do {} while(false)
#endif

#if UART_LOG
#define UART_CHARR LOG_CHARR
#define UART_STR LOG_STR
#else
#define UART_CHARR(...) do {} while(false)
#define UART_STR(...) do {} while(false)
#endif

#if UART_TRACE
#define UARTTR_CHARR LOG_CHARR
#define UARTTR_STR LOG_STR
#else
#define UARTTR_CHARR(...) do {} while(false)
#define UARTTR_STR(...) do {} while(false)
#endif

#if SOFD_LOG
#define SOFD_CHARR LOG_CHARR
#else
#define SOFD_CHARR(...) do {} while(false)
#endif

#if OVFL_LOG
#define OVFL_CHARR LOG_CHARR
#else
#define OVFL_CHARR(...) do {} while(false)
#endif

#if BLKF_LOG
#define BLKF_CHARR LOG_CHARR
#else
#define BLKF_CHARR(...) do {} while(false)
#endif



typedef struct
{
	float start, avg, /*floating,*/ M2, var, min, max, num, cur;
} StatValue;

/**
 * Resets the statistical value to the default state
 */
static inline void stats_reset(StatValue *stats)
{
	memset(stats, 0, sizeof(*stats));
}

/**
 * Updates the given statistical value with the new value
 */
static inline void stats_update(StatValue *stats, float val)
{
	if (stats->num == 0)
	{
		stats_reset(stats);
		stats->start = val;
		stats->min = val;
		stats->max = val;
		//stats->floating = val;
	}
	else
	{
		if (stats->min > val) stats->min = val;
		if (stats->max < val) stats->max = val;
	}
	stats->cur = val;
	// Welford algorithm
	stats->num++;
	float fac = 1.0f/stats->num;
	float diff = val - stats->avg;
	stats->avg += diff * fac;
	float diff2 = val - stats->avg;
	stats->M2 += diff * diff2;
	stats->var = stats->M2 * fac; // Sample Variance
	// Floating average
	/* if (Floating > 0)
	{
		int floatNum = stats->num < Floating? stats->num : Floating;
		stats->floating = (stats->floating*floatNum + val)/(floatNum+1);
	} */
}

/* Timing */

typedef uint64_t TimePoint;
typedef int32_t TimeSpan;

#if defined(CH32V307)

// CH32V307 has a nice 64bit SysTick that will count sub-us for 32 thousand years
#define TICKS_PER_US	(SYSCLKFRQ / 8)

static inline __attribute__((always_inline)) TimePoint GetTimePoint()
{ // Unit: 1/18 us
	return SysTick->CNT;
}

#elif defined(STM32)
// Generally only has a 24bit SysTick, will not even last 2 seconds
// So instead use TIM1 to get a us-Timer and use the interrupt to keep separate 64bit variable

#define TICKS_PER_US	1

// Global System Timer
extern volatile uint64_t usCounter;

static inline __attribute__((always_inline)) TimePoint GetTimePoint()
{ // Unit: us
	return usCounter + TIM1->CNT;
}

#endif

#define TICKS_PER_MS	(1000*TICKS_PER_US)

static inline __attribute__((always_inline)) TimePoint GetTimePointUS()
{
	return GetTimePoint() / TICKS_PER_US;
}

static inline __attribute__((always_inline)) TimePoint GetTimestampUS(TimePoint point)
{
	return point / TICKS_PER_US;
}

static inline __attribute__((always_inline)) TimeSpan GetUS(TimePoint point)
{
	return (point / TICKS_PER_US)%1000;
}

static inline __attribute__((always_inline)) TimeSpan GetMS(TimePoint point)
{
	return point / TICKS_PER_MS;
}

static inline __attribute__((always_inline)) TimeSpan GetTimeSpanUS(TimePoint pointA, TimePoint pointB)
{
	return (int64_t)(pointB - pointA) / TICKS_PER_US;
}

static inline __attribute__((always_inline)) TimeSpan GetTimeSpanMS(TimePoint pointA, TimePoint pointB)
{
	return (int64_t)(pointB - pointA) / TICKS_PER_MS;
}

static inline __attribute__((always_inline)) TimeSpan GetTimeSinceUS(TimePoint point)
{
	return GetTimeSpanUS(point, GetTimePoint());
}

static inline __attribute__((always_inline)) TimeSpan GetTimeSinceMS(TimePoint point)
{
	return GetTimeSpanMS(point, GetTimePoint());
}

static inline __attribute__((always_inline)) TimeSpan GetTimeUntilUS(TimePoint point)
{
	return GetTimeSpanUS(GetTimePoint(), point);
}

static inline __attribute__((always_inline)) TimeSpan GetTimeUntilMS(TimePoint point)
{
	return GetTimeSpanMS(GetTimePoint(), point);
}

static inline __attribute__((always_inline)) void delayUS(uint32_t us)
{ // General, works on all processors
	TimePoint tgt = GetTimePoint();
	tgt += us * TICKS_PER_US;
	while (GetTimePoint() < tgt);
}

static inline __attribute__((always_inline)) void delayMS(uint16_t ms)
{ // General, works on all processors
	delayUS(ms*1000);
}


#if defined(ENABLE_EVENTS)

#define EVENT_BUFFER_SIZE	(1<<12)
#define EVENT_BYTE_SIZE		4 // uint32_t
extern volatile uint_fast16_t eventHead, eventTail, eventSending;
extern uint8_t *eventBuffer; // [EVENT_BUFFER_SIZE];
extern uint8_t eventFilter[CONTROLLER_EVENT_MAX]; // Currently unused in favour of eventLogClass
extern uint8_t eventLogClass; // Alternative to full eventFilter. Currently only use this

__attribute__((unused))
static void LOG_EVT(enum ControllerEventID id, bool isNewEvent)
{
//	if (!eventFilter[id]) return;
	// Timestamps in 1/18us precision overflows every ~126years
	//uint32_t data = (GetTimePoint() << 8) | (id << 1) | (isNewEvent? 0x1 : 0x0);
	// In us
	uint32_t data = (GetTimePointUS() << 8) | (id << 1) | (isNewEvent? 0x1 : 0x0);

	USE_LOCKS();
	ATOMIC(
		if (eventHead >= EVENT_BUFFER_SIZE) eventHead = 0;
		*(uint32_t*)(eventBuffer+eventHead) = data;
		// WARNING: This assumes eventBuffer+eventHead is 4-byte aligned!
		// This chip cannot write uint32_t in one instruction to an unaligned address (and has no fallback either) 
		eventHead += 4;
	);
}

__attribute__((unused))
static void LOG_EVT_DATA(uint64_t d, uint_fast8_t id)
{
	uint32_t data = (d << 8) | id;

	USE_LOCKS();
	ATOMIC(
		if (eventHead >= EVENT_BUFFER_SIZE) eventHead = 0;
		*(uint32_t*)(eventBuffer+eventHead) = data;
		// WARNING: This assumes eventBuffer+eventHead is 4-byte aligned!
		// This chip cannot write uint32_t in one instruction to an unaligned address (and has no fallback either) 
		eventHead += 4;
	);
}

#define LOG_EVT_INT(...) { if (eventLogClass == 1) LOG_EVT(__VA_ARGS__); }
#define LOG_EVT_USB(...) { if (eventLogClass == 2) LOG_EVT(__VA_ARGS__); }
#define LOG_EVT_STR(...) { if (eventLogClass == 3) LOG_EVT(__VA_ARGS__); }
#define LOG_EVT_SET(...) { if (eventLogClass == 4) LOG_EVT(__VA_ARGS__); }
//#define LOG_EVT_INT(...) do {} while(false)
//#define LOG_EVT_USB(...) do {} while(false)
//#define LOG_EVT_STR(...) do {} while(false)

#else

#define LOG_EVT(...) do {} while(false)
#define LOG_EVT_DATA(...) do {} while(false)

#define LOG_EVT_INT(...) do {} while(false)
#define LOG_EVT_USB(...) do {} while(false)
#define LOG_EVT_STR(...) do {} while(false)
#define LOG_EVT_SET(...) do {} while(false)

#endif // EVENTLOG

#ifdef __cplusplus
}
#endif

#endif /* __UTIL_H */
