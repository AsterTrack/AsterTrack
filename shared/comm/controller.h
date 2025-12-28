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

#ifndef CONTROLLER_H
#define CONTROLLER_H

/**
 * This file describes the controller status and is shared between server and controller
 * Thus it needs to be kept in sync and working with all compilers (C style)!
*/

#include <stdint.h>
#include <stdbool.h>

enum ControllerCommState
{
	COMM_NO_CONN = 0,
	COMM_HAS_ID = 1,
	COMM_GOT_ACK = 2,
	COMM_MCU = 4,
	COMM_SBC = 8,
	COMM_READY = COMM_HAS_ID | COMM_GOT_ACK,
	COMM_MCU_READY = COMM_MCU | COMM_READY,
	COMM_SBC_READY = COMM_SBC | COMM_READY,
};

enum ControllerStatusFlags
{
	CONTROLLER_IDLE = 0,
	CONTROLLER_COMM_CHANNELS = 1,
	CONTROLLER_TIME_SYNC = 2
};

enum ControllerSyncConfig
{
	SYNC_CFG_NONE = 0,
	SYNC_CFG_GEN_RATE,
	SYNC_CFG_EXT_TRIG
	// Potentially many more to come
};

enum ControllerPortStatus
{
	PORT_INACTIVE = 0,
	PORT_POWERED = 1,
	PORT_SYNC_ENABLED = 2,
	PORT_CAM_STREAMING = 4
};

enum ControllerPowerState
{
	POWER_WAITING,
	POWER_PD_IN,
	POWER_EXT_IN,
	POWER_PD_IN_EXT_OUT
};

#endif // CONTROLLER_H