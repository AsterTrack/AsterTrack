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

#ifndef ANALOG_H
#define ANALOG_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

enum ControllerPowerState
{
	POWER_WAITING,
	POWER_PD_IN,
	POWER_EXT_IN,
	POWER_PD_IN_EXT_OUT
};

extern volatile enum ControllerPowerState powerInState;

void EnableADCs();
void DisableADCs();
bool AreADCsActive();

void SetAnalogWatchdogPDIn();
void SetAnalogWatchdogExtIn();
void SetAnalogWatchdogPDSrc();
void SetAnalogWatchdogExtSrc();
void SetAnalogWatchdogOff();

bool IsInInputRange(uint32_t mv);
uint32_t GetMillivoltsPD();
uint32_t GetMillivoltsExt();

bool EnablePowerPDIn();
bool EnablePowerExtIn();
bool EnablePowerExtOut();
void DisablePowerIn();

#ifdef __cplusplus
}
#endif

#endif /* __USB_DRIVER_H */
