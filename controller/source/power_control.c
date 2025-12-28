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

#include "ch32v30x_gpio.h"

#include "power_control.h"
#include "comm/controller.h"
#include "compat.h"
#include "util.h"

volatile enum ControllerPowerState powerInState;

void EnableADCs()
{
	// Wake up ADC from power-down mode
	if (!(ADC1->CTLR2 & ADC_ADON))
		ADC1->CTLR2 |= ADC_ADON;
	if (!(ADC2->CTLR2 & ADC_ADON))
		ADC2->CTLR2 |= ADC_ADON;

	// Need to wait 2 ADCCLK cycles between power on and calibration
	delayUS(1); // At 18 Mhz that is less than 1us

	// Reset and Start ADC Calibration
	ADC1->CTLR2 |= ADC_RSTCAL;
	ADC2->CTLR2 |= ADC_RSTCAL;
	while (ADC1->CTLR2 & ADC_RSTCAL);
	while (ADC2->CTLR2 & ADC_RSTCAL);
	ADC1->CTLR2 |= ADC_CAL;
	ADC2->CTLR2 |= ADC_CAL;
	while (ADC1->CTLR2 & ADC_CAL);
	while (ADC2->CTLR2 & ADC_CAL);

	// Start continuous conversion
	ADC1->CTLR2 |= ADC_ADON;
	ADC2->CTLR2 |= ADC_ADON;

	delayUS(5); // Wait at least 3.7us for first ADC sample
}

void DisableADCs()
{
	// Place ADC in power-down mode
	ADC1->CTLR2 &= ~ADC_ADON;
	ADC2->CTLR2 &= ~ADC_ADON;
}

bool AreADCsActive()
{
	return (ADC1->CTLR2 & ADC_ADON) && (ADC2->CTLR2 & ADC_ADON);
}

void SetAnalogWatchdogPDIn()
{
	ADC2->CTLR1 = 0;
	ADC1->CTLR1 = (ADC_AWDEN | ADC_AWDSGL | ADC_AWDIE | 9);
	// Set AWD thresholds
	ADC1->WDHTR = (21000 * 4095 / (11 * 3300));
	ADC1->WDLTR = (9000 * 4095 / (11 * 3300));
}

void SetAnalogWatchdogExtIn()
{
	ADC1->CTLR1 = 0;
	ADC2->CTLR1 = (ADC_AWDEN | ADC_AWDSGL | ADC_AWDIE | 8);
	// Set AWD thresholds
	ADC2->WDHTR = (25000 * 4095 / (11 * 3300));
	ADC2->WDLTR = (9000 * 4095 / (11 * 3300));
}

void SetAnalogWatchdogPDSrc()
{
	ADC1->CTLR1 = (ADC_AWDEN | ADC_AWDSGL | ADC_AWDIE | 9);
	// Set AWD thresholds
	ADC1->WDHTR = (10000 * 4095 / (11 * 3300));
	ADC1->WDLTR = 0;
}

void SetAnalogWatchdogExtSrc()
{
	ADC2->CTLR1 = (ADC_AWDEN | ADC_AWDSGL | ADC_AWDIE | 8);
	// Set AWD thresholds
	ADC2->WDHTR = (10000 * 4095 / (11 * 3300));
	ADC2->WDLTR = 0;
}

void SetAnalogWatchdogOff()
{
	ADC1->CTLR1 = 0;
	ADC2->CTLR1 = 0;
}

bool IsInInputRange(uint32_t mv)
{
	return mv > 9000 && mv < 25000;
}

uint32_t GetMillivoltsPD()
{
	return (ADC1->RDATAR & 0xFFF) * 11 * 3300 / 4095;
}

uint32_t GetMillivoltsExt()
{
	return (ADC2->RDATAR & 0xFFF) * 11 * 3300 / 4095;
}

bool EnablePowerPDIn()
{
	GPIO_RESET(GPIOE, GPIO_PIN_7);
	delayUS(1);
	if (!AreADCsActive() || !IsInInputRange(GetMillivoltsPD()))
	{ // PD Power does not fall into input range
		GPIO_RESET(GPIOE, GPIO_PIN_8);
		return false;
	}
	GPIO_SET(GPIOE, GPIO_PIN_8);
	return true;
}

bool EnablePowerExtIn()
{
	GPIO_RESET(GPIOE, GPIO_PIN_8);
	delayUS(1);
	if (!AreADCsActive() || !IsInInputRange(GetMillivoltsExt()))
	{ // Ext Power does not fall into input range
		GPIO_RESET(GPIOE, GPIO_PIN_7);
		return false;
	}
	GPIO_SET(GPIOE, GPIO_PIN_7);
	return true;
}

bool EnablePowerExtOut()
{
	if (!GPIO_READ(GPIOE, GPIO_PIN_8))
		return false;
	if (GPIO_READ(GPIOE, GPIO_PIN_7))
		return true;
	if (!AreADCsActive())
	{ // Should not happen
		POW_STR("/ADCs_Off_Err!");
		GPIO_RESET(GPIOE, GPIO_PIN_7);
		return false;
	}
	for (int i = 0; i < 10; i++)
	{ // Wait for voltage to drop if PD In and Ext Out was previously active
		if (GetMillivoltsExt() < 1000)
		{ // Likely not conflicting with other external power source
			GPIO_SET(GPIOE, GPIO_PIN_7);
			return true;
		}
		// Wait for next ADC sample (3.7us interval)
		while (!(ADC2->STATR & ADC_EOC));
	}
	// Failed to set Ext power out, but PD power in was accepted
	return false;
}

void DisablePowerIn()
{
	GPIO_RESET(GPIOE, GPIO_PIN_7);
	GPIO_RESET(GPIOE, GPIO_PIN_8);
}


/**
 * ADC Global Interrupt
 */
void ADC1_2_IRQHandler(void) __IRQ;
void ADC1_2_IRQHandler()
{
	if (ADC1->STATR & ADC_AWD)
	{
		if (powerInState == POWER_WAITING)
		{
			if (EnablePowerPDIn())
			{
				SetAnalogWatchdogPDIn();
				bool wantExtOut = false;
				if (wantExtOut && EnablePowerExtOut())
				{
					powerInState = POWER_PD_IN_EXT_OUT;
					POW_STR("/Got_PD_Pwr_ExtOut");
				}
				else
				{
					powerInState = POWER_PD_IN;
					POW_STR("/Got_PD_Pwr");
				}
			}
			else
				POW_STR("/Failed_PD_Pwr");
		}
		else
		{
			if (powerInState != POWER_PD_IN && powerInState != POWER_PD_IN_EXT_OUT)
				POW_STR("!INVALID_Ext");
			powerInState = POWER_WAITING;
			DisablePowerIn();
			SetAnalogWatchdogPDSrc();
			SetAnalogWatchdogExtSrc();
			POW_STR("/Lost_PD_Pwr");
		}
		ADC1->STATR &= ~ADC_AWD;
	}
	if (ADC2->STATR & ADC_AWD)
	{
		if (powerInState == POWER_WAITING)
		{
			if (EnablePowerExtIn())
			{
				powerInState = POWER_EXT_IN;
				SetAnalogWatchdogExtIn();
				POW_STR("/Got_Ext_Pwr");
			}
			else
				POW_STR("/Failed_Ext_Pwr");
		}
		else
		{
			if (powerInState != POWER_EXT_IN)
				POW_STR("!INVALID_Ext");
			powerInState = POWER_WAITING;
			DisablePowerIn();
			SetAnalogWatchdogPDSrc();
			SetAnalogWatchdogExtSrc();
			POW_STR("/Lost_Ext_Pwr");
		}
		ADC2->STATR &= ~ADC_AWD;
	}
}