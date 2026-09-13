/**
AsterTrack Optical Tracking System
Copyright (C) 2026 Seneral <seneral@seneral.dev> and contributors

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, specifically version 3.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef UI_SIGNALS_H
#define UI_SIGNALS_H

#include "util/trackdef.hpp"

#include <stdint.h>

typedef uint32_t CameraID; // util/eigendef.hpp

extern "C" {

enum ServerEvents : uint8_t
{
	EVT_MODE_SIMULATION_START,
	EVT_MODE_SIMULATION_STOP,
	EVT_MODE_DEVICE_START,
	EVT_MODE_DEVICE_STOP,
	EVT_START_STREAMING,
	EVT_STOP_STREAMING,
	EVT_DEVICE_DISCONNECT,
	EVT_UPDATE_CAMERAS,
	EVT_UPDATE_CALIBS,
	EVT_UPDATE_EVENTS,
	EVT_UPDATE_INTERFACE
};

enum CameraInteractEvent : uint8_t
{ // Event for CameraInteractState
	EVT_INTERACT_NONE		= 0,
	EVT_INTERACT_SELECTED	= 1 << 0,
	EVT_INTERACT_DESELECTED	= 1 << 1,
	EVT_INTERACT_FOCUSED	= 1 << 2,
	EVT_INTERACT_UNFOCUSED	= 1 << 3,
};

typedef bool (*InterfaceThread_t)();
typedef void (*SignalShouldClose_t)();
typedef void (*SignalLogUpdate_t)();
typedef void (*SignalCameraRefresh_t)(CameraID id);
typedef void (*SignalPipelineUpdate_t)();
typedef void (*SignalObservationReset_t)(FrameNum firstFrame);
typedef void (*SignalServerEvent_t)(ServerEvents event);
typedef void (*SignalCameraInteraction_t)(CameraID id, CameraInteractEvent event, bool affectAll);

}

/**
 * Signals to UI
 * (from Server and Pipeline)
 */

extern InterfaceThread_t InterfaceThread;
extern SignalShouldClose_t SignalInterfaceShouldClose;		// Signal: Server -> UI
extern SignalLogUpdate_t SignalLogUpdate;					// Signal: Server -> UI
extern SignalCameraRefresh_t SignalCameraRefresh;			// Signal: Server -> UI
extern SignalPipelineUpdate_t SignalPipelineUpdate;			// Signal: Pipeline -> UI
extern SignalObservationReset_t SignalObservationReset;		// Signal: Pipeline -> UI
extern SignalServerEvent_t SignalServerEvent;				// Signal: Server -> UI
extern SignalCameraInteraction_t SignalCameraInteraction;	// Signal: Server -> UI

#endif // UI_SIGNALS_H