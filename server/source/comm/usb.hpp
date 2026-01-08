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

#ifndef COMM_USB_H
#define COMM_USB_H

#include "util/util.hpp" // TimePoint_t

#include <atomic>
#include <cstddef>
#include <map>
#include <vector>
#include <mutex>
#include <memory> // shared_ptr

/**
 * USB Communication for input from cameras
 */


/* Structures */

// Forward-declared opaque structs
struct libusb_context_int;
struct libusb_state_int;

struct TransferStats
{
	int ep;
	uint32_t receiveCount;
	uint32_t receiveBytes;
	TimePoint_t lastReceive;
};

struct USBCommState
{
	libusb_state_int *driver_state = NULL;
	std::atomic<bool> deviceConnected = { false };
	std::atomic<bool> commStreaming = { false };
	std::atomic<int> streamingEndpoints = { 0 };
	bool isHighSpeed = false;

	bool recordStats = true;
	//std::mutex statMutex;
	TimePoint_t lastUSBStatCheck;
	std::map<void*, TransferStats> streamingEPStats;
	TimePoint_t lastUSBIntIn;
	float largestIntLag = 0;

	void *userState = NULL;
	std::shared_ptr<void> userDevice = NULL;
	void (*onControlResponse)(uint8_t request, uint16_t value, uint16_t index, uint8_t *data, int length, void *userState, std::shared_ptr<void> &userDevice, bool success) = NULL;
	void (*onUSBPacketIN)(uint8_t *data, int length, TimePoint_t receiveTime, uint8_t endpoint, void *userState, std::shared_ptr<void> &userDevice) = NULL;

/* 	void copyFrom(const USBCommState &other)
	{
		driver_state = other.driver_state;
		deviceConnected = other.deviceConnected.load();
		commStreaming = other.commStreaming.load();
		userState = other.userState;
		userDevice = other.userDevice;
		onControlResponse = other.onControlResponse;
		onUSBPacketIN = other.onUSBPacketIN;
	}
	void moveFrom(USBCommState &&other)
	{
		driver_state = other.driver_state;
		deviceConnected = std::move(other.deviceConnected);
		commStreaming = other.commStreaming.load();
		userState = other.userState;
		userDevice = other.userDevice;
		onControlResponse = other.onControlResponse;
		onUSBPacketIN = other.onUSBPacketIN;
	}
	USBCommState() = default;
	USBCommState(USBCommState &&other)
	{
		moveFrom(other);
	}
	USBCommState(const USBCommState &other)
	{
		copyFrom(other);
	}
	USBCommState& operator=(const USBCommState &other)
	{
		copyFrom(other);
		return *this;
	}
	USBCommState& operator=(USBCommState &&other)
	{
		moveFrom(std::move(other));
		return *this;
	} */
};


/* Functions */

// TODO: Redesign with shared_ptr<libusb_context_int> with custom deleter calling comm_exit_context
// but need to handle other shared_ptr<USBCommState> that may still reference context
libusb_context_int *comm_init_context();
void comm_exit_context(libusb_context_int *context);

std::vector<std::shared_ptr<USBCommState>> comm_connect(libusb_context_int *context);
void comm_disconnect(std::shared_ptr<USBCommState> state);

bool comm_startStream(std::shared_ptr<USBCommState> &state);
bool comm_stopStream(std::shared_ptr<USBCommState> &state);

int comm_check_free_control_requests(std::shared_ptr<USBCommState> &state);
int comm_check_free_control_datas(std::shared_ptr<USBCommState> &state);
int comm_submit_control_request(std::shared_ptr<USBCommState> &state, uint8_t request, uint16_t value, uint16_t index);
int comm_submit_control_data(std::shared_ptr<USBCommState> &state, uint8_t request, uint16_t value, uint16_t index, void* data = NULL, uint16_t size = 0);
void comm_cancel_control_request(std::shared_ptr<USBCommState> &state, int transfer);
void comm_cancel_control_data(std::shared_ptr<USBCommState> &state, int transfer);

#endif // COMM_USB_H