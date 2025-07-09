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

#ifndef UI_SIGNALS_H
#define UI_SIGNALS_H

#include <stdint.h>

typedef int CameraID; // util/eigendef.hpp

extern "C" {

enum ServerEvents : uint8_t {
	EVT_MODE_SIMULATION_START,
	EVT_MODE_SIMULATION_STOP,
	EVT_MODE_DEVICE_START,
	EVT_MODE_DEVICE_STOP,
	EVT_START_STREAMING,
	EVT_STOP_STREAMING,
	EVT_DEVICE_DISCONNECT,
	EVT_UPDATE_CAMERAS,
	EVT_UPDATE_OBSERVATIONS,
	EVT_UPDATE_CALIBS,
	EVT_UPDATE_EVENTS,
	EVT_UPDATE_INTERFACE
};

typedef bool (*InterfaceThread_t)();
typedef void (*SignalShouldClose_t)();
typedef void (*SignalLogUpdate_t)();
typedef void (*SignalCameraRefresh_t)(CameraID);
typedef void (*SignalPipelineUpdate_t)();
typedef void (*SignalServerEvent_t)(ServerEvents);

}

/**
 * Signals to UI
 * (from Server and Pipeline)
 */

extern InterfaceThread_t InterfaceThread;
extern SignalShouldClose_t SignalInterfaceShouldClose;	// Signal: Server -> UI
extern SignalLogUpdate_t SignalLogUpdate;				// Signal: Server -> UI
extern SignalCameraRefresh_t SignalCameraRefresh;		// Signal: Server -> UI
extern SignalPipelineUpdate_t SignalPipelineUpdate;		// Signal: Pipeline -> UI
extern SignalServerEvent_t SignalServerEvent;			// Signal: Server -> UI

#endif // UI_SIGNALS_H