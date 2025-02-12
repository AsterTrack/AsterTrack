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

//#define LOG_MAX_LEVEL LTrace

#include "tracking_controller.hpp"
#include "tracking_camera.hpp"

#include "comm/usb.hpp"

#include "util/log.hpp"

std::atomic<int> modeChangesWaiting, modeChangesConfirmed; // Allows to wait for multiple mode changes to complete

bool TrackingCameraState::sendPacket(PacketTag tag, uint8_t *data, unsigned int length)
{
	if (state.error.contextualRLock()->encountered)
		return false; // Cannot handle packet at this time, waiting for recovery
	if (controller)
	{
		comm_submit_control_data(controller->comm, COMMAND_OUT_SEND_PACKET, (uint16_t)tag, (uint16_t)(1<<port), data, (uint16_t)length);
	}
	/* else if (client && client->ready)
	{
		comm_writeHeader(*client, tag, (uint16_t)length);
		comm_write(*client, data, (uint16_t)length);
	} */
	else
		return false;
	return true;
}

void TrackingCameraState::sendModeSet(uint8_t mode)
{
	if (sendPacket(PACKET_CFG_MODE, &mode, 1))
	{
		setMode = (TrCamMode)mode;
		modeChangesWaiting.fetch_add(1);
	}
	else
	{
		LOG(LCameraDevice, LWarn, "Failed to send mode packet!");
	}
}

const char *getControllerEventName(ControllerEventID event)
{
	switch (event)
	{
	case CONTROLLER_INTERRUPT_USB:
		return "USB Interrupt";
	case CONTROLLER_INTERRUPT_UART:
		return "UART Interrupt";
	case CONTROLLER_INTERRUPT_SYNC_GEN:
		return "Sync Generation";
	case CONTROLLER_INTERRUPT_SYNC_INPUT:
		return "Sync Input";
	case CONTROLLER_INTERRUPT_LED_UPDATE:
		return "LED Update";
	case CONTROLLER_INTERRUPT_PD_INT:
		return "USBD-C PD Interrupt";
	case CONTROLLER_INTERRUPT_FLASH_BUTTON:
		return "Flash Button";
	case CONTROLLER_INTERRUPT_FLASH_TIMER:
		return "Flash Timer";

	case CONTROLLER_EVENT_USB_SOF:
		return "Start of Frame";
	case CONTROLLER_EVENT_USB_CONTROL:
		return "Control Transfer";
	case CONTROLLER_EVENT_USB_DATA_TX:
		return "Interrupt Transfer";
	
	case CONTROLLER_EVENT_USB_SENDING_NULL:
		return "TimeSync Packet";
	case CONTROLLER_EVENT_USB_SENDING_PACKET:
		return "Data Packet";
	case CONTROLLER_EVENT_USB_QUEUE_SOF:
		return "Queueing SOF Packet";

	case CONTROLLER_EVENT_SYNC:
		return "Sync Pulse";
	case CONTROLLER_EVENT_DATA_IN:
		return "Data received";
	case CONTROLLER_EVENT_DATA_OUT:
		return "Data sent over USB";
	default:
		return "Unknown Event";
	}
}