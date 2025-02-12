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

#include "comm/usb.hpp"

#include "util/log.hpp"

// For priority of USB thread
#if defined(__unix__)
#include <sys/resource.h>
#include <unistd.h>
#endif

#define DEBUG_DEVICE_DESC

#pragma warning(disable : 4200)
#include "libusb/libusb.h"

#include <iostream>
#include <thread>
#include <string.h>
#include <list>
#include <algorithm>
#include <cassert>


/**
 * USB Communication for input from cameras
 */

#define CTRL_TRANSFER_SIZE		2048		// Max control packet size. Maybe be transferred with multiple control transfers

#define INT_TRANSFER_SIZE		1024
#define INT_NUM_ENDPOINTS		8		// Maximum number of interrupt endpoints supported for an alternate setting
#define INT_NUM_TRANSFERS		8		// Concurrent transfers per interrupt endpoint

#define CTRL_NUM_TRANSFERS		32		// Maximum number of pending control transfers
#define CTRL_TIMEOUT			100

// Descriptor Defines
#define USBD_VID						5824 	// 0x16c0
#define USBD_PID						1500 	// 0x05dc
#define USBD_MANUFACTURER_STRING		"Seneral seneral.dev"
#define USBD_PRODUCT_STRING				"AsterTrack Controller" // Prefix, expecting versioning (e.g. V4) after
#define USBD_INTERFACE					0

struct libusb_context_int
{
	libusb_context *context;
	// Thread
	std::thread *usbEventHandlerThread;
	std::atomic<bool> usbEventHandlerRun;
	// Devices
	std::mutex deviceAccess;
	bool supportsHotplug;
	libusb_hotplug_callback_handle hotplug_handle;
	std::vector<libusb_device_handle*> potentialDevices;
	std::vector<std::pair<libusb_device_handle*, std::shared_ptr<USBCommState>>> connectedDevices;
};

struct libusb_state_int
{
	libusb_context_int *context;
	libusb_device_handle *devHandle;
	// Device configuration
	std::vector<std::pair<int, std::vector<uint8_t>>> intEndpointsAdresses;
	std::atomic<int> usbAltSetting;
	// Transfers
	bool blockTransfers;
	libusb_transfer *controlIN[CTRL_NUM_TRANSFERS];
	libusb_transfer *controlOUT[CTRL_NUM_TRANSFERS];
	std::atomic<bool> controlCancelFree;
	std::atomic<int> controlCancelCounter;
	libusb_transfer *intIN[INT_NUM_ENDPOINTS][INT_NUM_TRANSFERS];
	std::atomic<bool> streamCancelFree;
	std::atomic<int> intCancelCounter;
	int transferErrors = 0;
	// Transfer state
	std::atomic<bool> ctrlINPending[CTRL_NUM_TRANSFERS];
	std::atomic<bool> ctrlOUTPending[CTRL_NUM_TRANSFERS];
	std::atomic<bool> intINSubmitted[INT_NUM_ENDPOINTS][INT_NUM_TRANSFERS];
	// Buffers
	alignas(2) uint8_t ctrlINBuf[CTRL_NUM_TRANSFERS][CTRL_TRANSFER_SIZE+LIBUSB_CONTROL_SETUP_SIZE+1];
	alignas(2) uint8_t ctrlOUTBuf[CTRL_NUM_TRANSFERS][CTRL_TRANSFER_SIZE+LIBUSB_CONTROL_SETUP_SIZE+1];
	alignas(2) uint8_t intINBuf[INT_NUM_ENDPOINTS][INT_NUM_TRANSFERS][INT_TRANSFER_SIZE];
};

static int comm_hotplug_callback(libusb_context *ctx, libusb_device *dev, libusb_hotplug_event event, void *user_data);
static void usbEventHandler(libusb_context_int *libusb);

static void onInterruptIN(libusb_transfer *transfer);
static void onControlSent(libusb_transfer *transfer);
static void onControlResponse(libusb_transfer *transfer);

libusb_context_int *comm_init_context()
{
	libusb_context_int *libusb_context = new libusb_context_int();
	//memset(libusb_context, 0, sizeof(*libusb_context));
	// Init context
	if (libusb_init(&libusb_context->context) == 0)
	{ // Successful init
		libusb_set_option(libusb_context->context, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING);
	}
	else
	{ // Unsuccessful
		libusb_context->context = NULL;
		return libusb_context;
	}
	// Start usb thread
	libusb_context->usbEventHandlerRun = true;
	libusb_context->usbEventHandlerThread = new std::thread(usbEventHandler, libusb_context);
	// Register hotplug
	libusb_context->supportsHotplug = false; // libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG);
	// TODO: Fix libusb hotplug on linux
	// Windows just outright doesn't support it anyway
	// TODO: Implement new controller detection somehow if hotplug doesn't do it. Iteration takes too long to just do constantly
	// So currently relying on manually connecting to controller (entering device mode)
	if (libusb_context->supportsHotplug)
		libusb_hotplug_register_callback(libusb_context->context, 
			LIBUSB_HOTPLUG_MATCH_ANY, LIBUSB_HOTPLUG_ENUMERATE, 
			USBD_VID, USBD_PID, 0x00, 
			comm_hotplug_callback, libusb_context, &libusb_context->hotplug_handle);
	return libusb_context;
}

void comm_exit_context(libusb_context_int *libusb_context)
{
	if (libusb_context == NULL) return;
	libusb_context->usbEventHandlerRun = false;
	while (!libusb_context->connectedDevices.empty())
		comm_disconnect(libusb_context->connectedDevices.back().second);
	for (auto &dev : libusb_context->potentialDevices)
		libusb_close(dev);
	if (libusb_context->supportsHotplug)
		libusb_hotplug_deregister_callback(libusb_context->context, libusb_context->hotplug_handle);
	else
		libusb_interrupt_event_handler(libusb_context->context);
	if (libusb_context->usbEventHandlerThread)
	{
		if (libusb_context->usbEventHandlerThread->joinable())
		{
			libusb_context->usbEventHandlerThread->join();
		}
		delete libusb_context->usbEventHandlerThread;
	}
	delete libusb_context;
}

static int comm_hotplug_callback(libusb_context *context, libusb_device *device, libusb_hotplug_event event, void *user_data)
{
	// dev has correct VID & PID
	libusb_context_int *libusb_context = (libusb_context_int*)user_data;

	// device has correct VID & PID, no need to check
	//libusb_device_descriptor devDesc;
	//libusb_get_device_descriptor(device, &devDesc);

	if (LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED == event)
	{
		libusb_device_handle *devHandle;
		int ret = libusb_open(device, &devHandle);
		if (ret != 0)
		{ // On linux, this shouldn't happen. On Windows, this means it's already claimed, but windows doesn't support hotplugging
			LOG(LUSB, LError, "Failed to open device: %s!\n", libusb_error_name(ret));
			return 0;
		}
		libusb_context->deviceAccess.lock();
		bool recorded = false;
		for (int i = 0; i < libusb_context->potentialDevices.size(); i++)
			if (libusb_get_device(libusb_context->potentialDevices[i]) == device)
				recorded = true; // Already enumerated, can happen due to synchronisation issues in libusb
		if (!recorded) // Found new potential controller - can not check for strings in callback
			libusb_context->potentialDevices.push_back(devHandle);
		libusb_context->deviceAccess.unlock();
	}
	else if (LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT == event)
	{
		libusb_context->deviceAccess.lock();
		for (auto dev = libusb_context->potentialDevices.begin(); dev != libusb_context->potentialDevices.end();)
		{
			if (libusb_get_device(*dev) == device)
			{
				libusb_close(*dev);
				dev = libusb_context->potentialDevices.erase(dev);
			}
			else dev++;
		}
		for (auto dev = libusb_context->connectedDevices.begin(); dev != libusb_context->connectedDevices.end();)
		{
			if (libusb_get_device(dev->first) == device)
			{ // Already connected to device, notify server
				LOG(LUSB, LDebug, "Device was disconnected (callback)!\n");
				dev->second->deviceConnected = false;
			}
			else dev++;
		}
		libusb_context->deviceAccess.unlock();
	}
	return 0;
}
static bool comm_cancelControlTransfers(libusb_state_int *libusb, bool freeTransfers)
{
	// Cancel transfers and wait for them to be freed
	libusb->blockTransfers = true;
	libusb->controlCancelFree = freeTransfers;
	libusb->controlCancelCounter = 0;
	int expectedCancelled = 0;
	for (int i = 0; i < CTRL_NUM_TRANSFERS; i++)
	{
		int ret;
		if (libusb->ctrlINPending[i])
		{
			expectedCancelled++;
			if ((ret = libusb_cancel_transfer(libusb->controlIN[i])) != 0)
				LOG(LUSB, LDebug, "Could not cancel control IN transfer %p with code %d!", libusb->controlIN[i], ret)
			else
				LOG(LUSB, LTrace, "Control IN transfer %p will be cancelled!", libusb->controlIN[i])
		}
		else
		{
			LOG(LUSB, LTrace, "Control IN transfer %p was not submitted!", libusb->controlIN[i])
			if (freeTransfers)
				libusb_free_transfer(libusb->controlIN[i]);
		}

		if (libusb->ctrlOUTPending[i])
		{
			expectedCancelled++;
			if ((ret = libusb_cancel_transfer(libusb->controlOUT[i])) != 0)
				LOG(LUSB, LDebug, "Could not cancel control OUT transfer %p with code %d!", libusb->controlOUT[i], ret)
			else
				LOG(LUSB, LTrace, "Control OUT transfer %p will be cancelled!", libusb->controlOUT[i])
		}
		else
		{
			LOG(LUSB, LTrace, "Control OUT transfer %p was not submitted!", libusb->controlOUT[i])
			if (freeTransfers)
				libusb_free_transfer(libusb->controlOUT[i]);
		}
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(50));
	if (libusb->controlCancelCounter != expectedCancelled)
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	if (libusb->controlCancelCounter != expectedCancelled)
		LOG(LUSB, LWarn, "Could not cleanly cancel control transfers, cancelled %d of %d expected.\n", 
			libusb->controlCancelCounter.load(), expectedCancelled)
	if (freeTransfers)
	{
		for (int i = 0; i < CTRL_NUM_TRANSFERS; i++)
		{
			libusb->controlIN[i] = NULL;
			libusb->controlOUT[i] = NULL;
		}
	}
	libusb->controlCancelFree = false;
	libusb->blockTransfers = false;
	return libusb->controlCancelCounter == CTRL_NUM_TRANSFERS*2;
}
static void comm_cancelStreamTransfers(libusb_state_int *libusb, bool freeTransfers)
{
	// Cancel transfers and wait for them to be freed
	libusb->streamCancelFree = freeTransfers;

	libusb->intCancelCounter = 0;
	int expectedCancelled = 0;
	int manuallyCancelled = 0;
	for (int i = 0; i < INT_NUM_ENDPOINTS; i++)
		for (int j = 0; j < INT_NUM_TRANSFERS; j++)
			if (libusb->intIN[i][j])
			{
				expectedCancelled++;
				int ret = libusb_cancel_transfer(libusb->intIN[i][j]);
				if (ret == LIBUSB_ERROR_NOT_FOUND)
				{ // Still pending
					LOG(LUSB, LWarn, "Interrupt transfer %p has been blocked and is considered cancelled!\n", libusb->intIN[i][j]);
					libusb->intCancelCounter++;
					// TODO: Previously could result in a crash because a later event handler still assumes this transfer to exist. Verify this isn't the case anymore.
					if (freeTransfers)
						libusb_free_transfer(libusb->intIN[i][j]);
					manuallyCancelled++;
				}
				else if (ret == LIBUSB_SUCCESS)
					LOG(LUSB, LTrace, "Interrupt transfer %p will be cancelled!\n", libusb->intIN[i][j])
				else
					LOG(LUSB, LError, "Failed to cancel interrupt transfer %p with error code %d\n", libusb->intIN[i][j], ret)
			}

	std::this_thread::sleep_for(std::chrono::milliseconds(50));
	if (libusb->intCancelCounter != expectedCancelled)
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	if (libusb->intCancelCounter != expectedCancelled)
		LOG(LUSB, LWarn, "Could not cleanly cancel interrupt transfers, cancelled %d of %d expected, of which %d were forcefully cancelled.\n",
			libusb->intCancelCounter.load(), expectedCancelled, manuallyCancelled)
	if (freeTransfers)
	{
		for (int i = 0; i < INT_NUM_ENDPOINTS; i++)
			for (int j = 0; j < INT_NUM_TRANSFERS; j++)
				libusb->intIN[i][j] = NULL;
	}

	libusb->streamCancelFree = false;
}

std::vector<std::shared_ptr<USBCommState>> comm_connect(libusb_context_int *libusb_context)
{
	if (libusb_context == NULL) return {};
	if (libusb_context->supportsHotplug && libusb_context->potentialDevices.empty()) return {};
	if (libusb_context->context == NULL) return {};

	libusb_context->deviceAccess.lock();
	// Filter devices based on strings (required by free VID & PID)
	static char strBuf[256];
	if (!libusb_context->supportsHotplug)
	{ // Not hotplug list, list all devices and test for VID & PID
		libusb_device **devList;
		// TODO: This takes 1.2 seconds on windows, not suitable for constant checking to detect new controller
		ssize_t devCount = libusb_get_device_list(libusb_context->context, &devList);
		if (devCount < 0)
		{
			LOG(LUSB, LError, "Failed to list devices with error code %d!\n", (int)devCount);
		}

		for (int i = 0; i < devCount; i++)
		{
			libusb_device *device = devList[i];
			libusb_device_descriptor devDesc;
			if (libusb_get_device_descriptor(device, &devDesc) < 0) continue;
			if (devDesc.idVendor == USBD_VID && devDesc.idProduct == USBD_PID)
			{ // Candidate found - check with Manufacturer String match
				// TODO: Check if already connected to device
				//std::this_thread::sleep_for(std::chrono::milliseconds(10)); // get_usbfs_fd error suggests 10ms of waiting
				libusb_device_handle *devHandle;
				int ret = libusb_open(device, &devHandle);
				if (ret != 0)
				{ // On Windows, this means it's already claimed. On linux, this shouldn't happen
					if (ret == LIBUSB_ERROR_ACCESS)
						LOG(LUSB, LError, "Lacking permissions to open controller USB device!\n")
					else
						LOG(LUSB, LError, "Unknown error while trying to open controller USB device!\n")
					continue;
				}
				libusb_context->potentialDevices.push_back(devHandle);
			}
		}
		// Frees devices that have not been selected
		if (devCount >= 0)
			libusb_free_device_list(devList, 1);
	}

	// Check for strings
	for (auto dev = libusb_context->potentialDevices.begin(); dev != libusb_context->potentialDevices.end();)
	{
		libusb_device_descriptor devDesc;
		if (libusb_get_device_descriptor(libusb_get_device(*dev), &devDesc) == 0)
		{
			int err = libusb_get_string_descriptor_ascii(*dev, devDesc.iManufacturer, (unsigned char*)strBuf, sizeof(strBuf));
			if (err <= 0)
				LOG(LUSB, LError, "Failed to get manufacturer string from potential controller USB device, error code %d\n", err)
			else if (strcmp((const char*)strBuf, USBD_MANUFACTURER_STRING) == 0)
			{
				err = libusb_get_string_descriptor_ascii(*dev, devDesc.iProduct, (unsigned char*)strBuf, sizeof(strBuf));
				if (err <= 0)
					LOG(LUSB, LError, "Failed to get product string from potential controller USB device, error code %d\n", err)
				else if (strncmp((const char*)strBuf, USBD_PRODUCT_STRING, sizeof(USBD_PRODUCT_STRING)-1) == 0)
				{ // Use strncmp to exclude null delimiter -> startsWith
					dev++;
					continue;
				}
				else
					LOG(LUSB, LWarn, "USB Device had different product string '%s'! Expected '%s'!\n", strBuf, USBD_PRODUCT_STRING)
			}
			else
				LOG(LUSB, LWarn, "USB Device had different manufacturer '%s'! Expected '%s'!\n", strBuf, USBD_MANUFACTURER_STRING)
		}
		else
			LOG(LUSB, LWarn, "Failed to get device descriptor of potential controller USB device!\n");
		libusb_close(*dev);
		dev = libusb_context->potentialDevices.erase(dev);
	}

	// Connect to all devices
	std::vector<std::shared_ptr<USBCommState>> addedDevices;
	for (auto dev : libusb_context->potentialDevices)
	{
		// Claim device interface
		int code;
		if (libusb_kernel_driver_active(dev, USBD_INTERFACE) == 1)
			libusb_detach_kernel_driver(dev, USBD_INTERFACE);
		if ((code = libusb_claim_interface(dev, USBD_INTERFACE)) != 0)
		{ // Already connected to (most likely by another instance)
			libusb_close(dev);
			continue;
		}
		libusb_device *device = libusb_get_device(dev);

		// Init CommState
		std::shared_ptr<USBCommState> state = std::make_shared<USBCommState>();
		state->driver_state = new libusb_state_int();
		state->driver_state->context = libusb_context;
		state->driver_state->devHandle = dev;
		libusb_state_int *libusb = state->driver_state;

		// Config and interface overview
		libusb_config_descriptor *configDesc;
		libusb_get_active_config_descriptor(device, &configDesc);
		const libusb_interface &interface = configDesc->interface[USBD_INTERFACE];
		libusb->intEndpointsAdresses.reserve(interface.num_altsetting);
		for (int a = 0; a < interface.num_altsetting; a++)
		{
			const libusb_interface_descriptor &interfaceDesc = interface.altsetting[a];
			if (interfaceDesc.bNumEndpoints > 0 && (interfaceDesc.endpoint[0].bmAttributes & 0b11) == LIBUSB_TRANSFER_TYPE_INTERRUPT)
			{ // Alternate setting is interrupt-based
				libusb->intEndpointsAdresses.push_back({ a, {} });
			}

	#ifdef DEBUG_DEVICE_DESC
			LOG(LUSB, LInfo, "Interface %d - %d (C %d, SC %d, P %d) has %d endpoints!\n", interfaceDesc.bInterfaceNumber, interfaceDesc.bAlternateSetting, interfaceDesc.bInterfaceClass, interfaceDesc.bInterfaceSubClass, interfaceDesc.bInterfaceProtocol, interfaceDesc.bNumEndpoints);
	#endif

			for (int e = 0; e < interfaceDesc.bNumEndpoints; e++)
			{
				const libusb_endpoint_descriptor &epDesc = interfaceDesc.endpoint[e];
				libusb_transfer_type type = (libusb_transfer_type)(epDesc.bmAttributes & 0b11);
				if (type == LIBUSB_TRANSFER_TYPE_INTERRUPT)
				{ // Register interrupt endpoint as part of this alternate setting
					libusb->intEndpointsAdresses.back().second.push_back(epDesc.bEndpointAddress);
				}
			}
		}
		libusb_free_config_descriptor(configDesc);

		std::sort(libusb->intEndpointsAdresses.begin(), libusb->intEndpointsAdresses.end(), [](const auto &a, const auto &b) { return a.second.size() > b.second.size(); });

		// Setup generic control request expecting a response with data from device
		for (int i = 0; i < CTRL_NUM_TRANSFERS; i++)
		{
			libusb_fill_control_setup(libusb->ctrlINBuf[i], LIBUSB_RECIPIENT_INTERFACE | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN, 0x00, 0x0000, USBD_INTERFACE, CTRL_TRANSFER_SIZE);
			libusb->controlIN[i] = libusb_alloc_transfer(0);
			libusb_fill_control_transfer(libusb->controlIN[i], libusb->devHandle, libusb->ctrlINBuf[i], &onControlResponse, state.get(), CTRL_TIMEOUT);
			libusb->ctrlINPending[i] = false;
		}

		// Setup first generic control request sending data to device
		for (int i = 0; i < CTRL_NUM_TRANSFERS; i++)
		{
			libusb_fill_control_setup(libusb->ctrlOUTBuf[i], LIBUSB_RECIPIENT_INTERFACE | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT, 0x00, 0x0000, USBD_INTERFACE, CTRL_TRANSFER_SIZE);
			libusb->controlOUT[i] = libusb_alloc_transfer(0);
			libusb_fill_control_transfer(libusb->controlOUT[i], libusb->devHandle, libusb->ctrlOUTBuf[i], &onControlSent, state.get(), CTRL_TIMEOUT);
			libusb->ctrlOUTPending[i] = false;
		}

		// Get device speed
		int speed = libusb_get_device_speed(device);
		if (speed == LIBUSB_SPEED_HIGH)
			state->isHighSpeed = true;
		else if (speed == LIBUSB_SPEED_FULL)
			state->isHighSpeed = false;
		else if (speed == LIBUSB_SPEED_UNKNOWN)
			LOG(LUSB, LError, "Connected controller has unknown speed!\n")
		else
			LOG(LUSB, LError, "Connected controller has unrecognised speed %d!\n", speed)

		libusb->usbAltSetting = -1;
		libusb->blockTransfers = false;
		state->commStreaming = false;
		state->deviceConnected = true;

		// Add to device list
		libusb_context->connectedDevices.emplace_back(dev, state); // new shared_ptr
		addedDevices.push_back(std::move(state));
	}

	libusb_context->potentialDevices.clear();
	libusb_context->deviceAccess.unlock();
	return addedDevices;
}
void comm_disconnect(std::shared_ptr<USBCommState> &state)
{
	if (state->driver_state == NULL) return;
	libusb_state_int *libusb = state->driver_state;
	libusb_context_int *libusb_context = libusb->context;
	bool connected = state->deviceConnected;
	state->deviceConnected = false;
	state->userDevice = nullptr;
	// Disconnect comms
	comm_stopStream(state);
	// Cancel and free control transfers
	comm_cancelControlTransfers(libusb, true);
	// Release interface
	libusb_release_interface(libusb->devHandle, USBD_INTERFACE);

	libusb_context->deviceAccess.lock();
	// Store device handle incase it is still connected
	if (connected)
	{ // TODO: Mark to not auto-reconnect for future manual disconnections
		libusb_context->potentialDevices.push_back(libusb->devHandle);
	}
	else
	{
		libusb_close(libusb->devHandle);
	}
	// Remove record of device
	for (auto dev = libusb_context->connectedDevices.begin(); dev != libusb_context->connectedDevices.end(); dev++)
	{
		if (dev->first == libusb->devHandle)
		{
			delete state->driver_state;
			libusb_context->connectedDevices.erase(dev);
			break;
		}
	}
	libusb_context->deviceAccess.unlock();
}

bool comm_startStream(std::shared_ptr<USBCommState> &state)
{
	int code;

	if (!state->driver_state)
		return false;
	libusb_state_int *libusb = state->driver_state;
	if (libusb->usbAltSetting != -1)
		comm_stopStream(state);

	if (!state->deviceConnected)
		return false;

#ifdef _WIN32
	// WinUsb_SetCurrentAlternateSetting requires no ongoing transfers...
	comm_cancelControlTransfers(libusb, false);
#endif

	std::vector<unsigned char> endpoints;
	for (auto &it : libusb->intEndpointsAdresses)
	{
		if (it.second.size() > INT_NUM_ENDPOINTS)
			continue;
		if ((code = libusb_set_interface_alt_setting(libusb->devHandle, USBD_INTERFACE, it.first)) != 0)
		{
			LOG(LUSB, LError, "Failed to set alternate setting %d with %d interrupt endpoints: %s\n", it.first, (int)it.second.size(), libusb_error_name(code));
			continue;
		}
		libusb->usbAltSetting = it.first;
		state->streamingEndpoints.store(it.second.size());
		endpoints = it.second;
		LOG(LUSB, LDebug, "Set alternate setting %d with %d interrupt endpoints!\n", it.first, (int)it.second.size());
		break;
	}
	if (libusb->usbAltSetting == -1)
	{
		LOG(LUSB, LError, "Failed to set any interrupt alternate setting!\n");
		return false;
	}

	// Setup interrupt transfers
	for (int i = 0; i < endpoints.size(); i++)
	{
		for (int j = 0; j < INT_NUM_TRANSFERS; j++)
		{
			libusb->intIN[i][j] = libusb_alloc_transfer(0);
			libusb_fill_interrupt_transfer(libusb->intIN[i][j], libusb->devHandle, endpoints[i], libusb->intINBuf[i][j], INT_TRANSFER_SIZE, &onInterruptIN, state.get(), 1000);
		}
	}

	// Start submitting transfers
	for (int i = 0; i < endpoints.size(); i++)
	{
		for (int j = 0; j < INT_NUM_TRANSFERS; j++)
		{
			int code = libusb_submit_transfer(libusb->intIN[i][j]);
			LOG(LUSB, LTrace, "Submit interrupt transfer for ep %d, num %d: %s!\n", endpoints[i], j, libusb_error_name(code));
		}
	}

	state->commStreaming = true;
	return true;
}

bool comm_stopStream(std::shared_ptr<USBCommState> &state)
{
	if (!state->commStreaming) return true;
	state->commStreaming = false;
	libusb_state_int *libusb = state->driver_state;

	// Cancel and free stream transfers
	comm_cancelStreamTransfers(libusb, true);

	int code = libusb_set_interface_alt_setting(libusb->devHandle, USBD_INTERFACE, 0);
	if (code != 0)
	{
		LOG(LUSB, LWarn, "Failed to reset device interface!\n");
	}

	libusb->usbAltSetting = -1;
	return true;
}

int comm_check_free_control_requests(std::shared_ptr<USBCommState> &state)
{
	if (!state->deviceConnected)
	{
		LOG(LUSB, LWarn, "Failed to submit because device is disconnected!\n");
		return -1;
	}
	libusb_state_int *libusb = state->driver_state;
	int pending = 0;
	for (int t = 0; t < CTRL_NUM_TRANSFERS; t++)
		if (libusb->ctrlINPending[t]) pending++;
	return CTRL_NUM_TRANSFERS-pending;
}

int comm_check_free_control_datas(std::shared_ptr<USBCommState> &state)
{
	if (!state->deviceConnected)
	{
		LOG(LUSB, LWarn, "Failed to submit because device is disconnected!\n");
		return -1;
	}
	libusb_state_int *libusb = state->driver_state;
	int pending = 0;
	for (int t = 0; t < CTRL_NUM_TRANSFERS; t++)
		if (libusb->ctrlOUTPending[t]) pending++;
	return CTRL_NUM_TRANSFERS-pending;
}

int comm_submit_control_request(std::shared_ptr<USBCommState> &state, uint8_t request, uint16_t value, uint16_t index)
{
	if (!state->deviceConnected)
	{
		LOG(LUSB, LWarn, "Failed to submit because device is disconnected!\n");
		return -1;
	}
	libusb_state_int *libusb = state->driver_state;
	if (libusb->blockTransfers)
	{
		LOG(LUSB, LWarn, "Failed to submit because transfers are blocked!\n");
		return -1;
	}
	int t = 0;
	for (; t < CTRL_NUM_TRANSFERS; t++)
		if (!libusb->ctrlINPending[t]) break;
	// Detect USB soft stall (control)
	static int pendingFull = 0;
	if (t == CTRL_NUM_TRANSFERS)
	{
		if (pendingFull++ != 10)
		{
			LOG(LUSB, LWarn, "Failed to submit control IN, no free control transfer! Transfers Pending: %d\n", CTRL_NUM_TRANSFERS);
			return -1;
		}
		// cancel the transfers, suspected to be stalled
		LOG(LUSB, LWarn, "Cancelling %d control IN transfers due to stalling, lost 10 more waiting for them!\n", CTRL_NUM_TRANSFERS);
		libusb->controlCancelFree = false; // Don't free transfers after cancelling
		for (int i = 0; i < CTRL_NUM_TRANSFERS; i++)
			libusb_cancel_transfer(libusb->controlIN[i]);
		return -1;
	}
	pendingFull = 0;
	// Setup and send of control transfer
	libusb_fill_control_setup(libusb->ctrlINBuf[t], LIBUSB_RECIPIENT_OTHER | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN, request, value, index, CTRL_TRANSFER_SIZE);
	int ret = libusb_submit_transfer(libusb->controlIN[t]);
	if (ret != 0)
	{
		if (ret == LIBUSB_ERROR_NO_DEVICE)
		{
			LOG(LUSB, LDebug, "Device was disconnected (submit control IN transfer)!\n");
			state->deviceConnected = false;
		}
		else
			LOG(LUSB, LError, "Failed to submit control request: %d (%s)!\n", ret, libusb_error_name(ret));
		return -1;
	}
	libusb->ctrlINPending[t] = true;
	return t;
}

int comm_submit_control_data(std::shared_ptr<USBCommState> &state, uint8_t request, uint16_t value, uint16_t index, void* data, uint16_t size)
{
	assert(size <= CTRL_TRANSFER_SIZE);
	if (!state->deviceConnected)
		return -1;
	libusb_state_int *libusb = state->driver_state;
	if (libusb->blockTransfers)
		return -1;
	int t = 0;
	for (; t < CTRL_NUM_TRANSFERS; t++)
		if (!libusb->ctrlOUTPending[t]) break;
	// Detect USB soft stall (control)
	static int pendingFull = 0;
	if (t == CTRL_NUM_TRANSFERS)
	{
		if (pendingFull++ != 10)
		{
			LOG(LUSB, LWarn, "Failed to submit control OUT, no free control transfer! Transfers Pending: %d\n", CTRL_NUM_TRANSFERS);
			return -1;
		}
		// cancel the transfers, suspected to be stalled
		LOG(LUSB, LWarn, "Cancelling %d control OUT transfers due to stalling, lost 10 more waiting for them!\n", CTRL_NUM_TRANSFERS);
		libusb->controlCancelFree = false; // Don't free transfers after cancelling
		for (int i = 0; i < CTRL_NUM_TRANSFERS; i++)
			libusb_cancel_transfer(libusb->controlOUT[i]);
		return -1;
	}
	pendingFull = 0;
	if (data != NULL && size != 0)
		memcpy(libusb->ctrlOUTBuf[t]+8, data, size);
	libusb_fill_control_setup(libusb->ctrlOUTBuf[t], LIBUSB_RECIPIENT_OTHER | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT, request, value, index, size);
	int ret = libusb_submit_transfer(libusb->controlOUT[t]);
	if (ret != 0)
	{
		if (ret == LIBUSB_ERROR_NO_DEVICE)
		{
			LOG(LUSB, LDebug, "Device was disconnected (submit control OUT transfer)!\n");
			state->deviceConnected = false;
		}
		else
			LOG(LUSB, LError, "Failed to submit control data: %d (%s)!\n", ret, libusb_error_name(ret));
		return -1;
	}
	libusb->ctrlOUTPending[t] = true;
	return t;
}

void comm_cancel_control_request(std::shared_ptr<USBCommState> &state, int transfer)
{
	if (transfer < 0 || transfer >= CTRL_NUM_TRANSFERS)
		return;
	libusb_state_int *libusb = state->driver_state;
	if (libusb->ctrlINPending[transfer])
	{
		libusb->controlCancelFree = false; // Don't free transfers after cancelling
		libusb_cancel_transfer(libusb->controlIN[transfer]);
	}
}

void comm_cancel_control_data(std::shared_ptr<USBCommState> &state, int transfer)
{
	if (transfer < 0 || transfer >= CTRL_NUM_TRANSFERS)
		return;
	libusb_state_int *libusb = state->driver_state;
	if (libusb->ctrlOUTPending[transfer])
	{
		libusb->controlCancelFree = false; // Don't free transfers after cancelling
		libusb_cancel_transfer(libusb->controlOUT[transfer]);
	}
}

static int handleTransferStatus(libusb_transfer *transfer, std::string label, bool isControl, bool resend = true)
{
	USBCommState *state = (USBCommState*)transfer->user_data;
	libusb_state_int *libusb = state->driver_state;
	switch (transfer->status)
	{
	case LIBUSB_TRANSFER_CANCELLED:
		if (isControl)
		{
			LOG(LUSB, LTrace, "Cancelled %scontrol transfer %p!\n", libusb->controlCancelFree? "and freed " : "", transfer);
			if (libusb->controlCancelFree)
				libusb_free_transfer(transfer);
			libusb->controlCancelCounter++;
		}
		else
		{
			LOG(LUSB, LTrace, "Cancelled %sinterrupt transfer %p!\n", libusb->streamCancelFree? "and freed " : "", transfer);
			if (libusb->streamCancelFree)
				libusb_free_transfer(transfer);
			libusb->intCancelCounter++;
		}
		return -1;
	case LIBUSB_TRANSFER_NO_DEVICE:
		// Drop connection immediately
		LOG(LUSB, LDebug, "Device was disconnected (transfer code)!\n");
		state->deviceConnected = false;
		return -1;
	case LIBUSB_TRANSFER_COMPLETED:
		libusb->transferErrors = 0;
		return 0;
	case LIBUSB_TRANSFER_TIMED_OUT:
		LOG(LUSB, LTrace, "USB transfer for ep %d timed out!\n", transfer->endpoint);
		if (resend)
	 		libusb_submit_transfer(transfer);
		return 1;
	case LIBUSB_TRANSFER_ERROR:
	case LIBUSB_TRANSFER_OVERFLOW:
	case LIBUSB_TRANSFER_STALL:
	default:
		LOG(LUSB, LWarn, "%s completed with status code %d (%s)!\n", label.c_str(), transfer->status, libusb_error_name(transfer->status));
		libusb->transferErrors++;
		if (libusb->transferErrors > 10)
		{
			LOG(LUSB, LError, "Errors indicate comm loss!\n");
			state->deviceConnected = false;
			libusb->transferErrors = 0;
			return -1;
		}
		if (resend)
	 		libusb_submit_transfer(transfer);
		return 1;
	}
}

static void onControlSent(libusb_transfer *transfer)
{
	USBCommState *state = (USBCommState*)transfer->user_data;
	libusb_state_int *libusb = state->driver_state;
	struct libusb_control_setup *setup = (struct libusb_control_setup*)transfer->buffer;

	if (transfer->status != 0)
	{
		LOG(LUSB, LDebug, "Control OUT Req (%hhu, %hu, %hu) returned with status %d!\n", setup->bRequest, setup->wValue, setup->wIndex, (int)transfer->status);
	}

	bool transparentResend = true;
	int res = handleTransferStatus(transfer, asprintf_s("Control OUT Transfer (%hhu, %hu, %hu)", setup->bRequest, setup->wValue, setup->wIndex), true, transparentResend);
	bool finalHandling = res != 1 || !transparentResend;

	if (finalHandling)
	{ // Not already re-sent
		for (int i = 0; i < CTRL_NUM_TRANSFERS; i++)
		{
			if (libusb->controlOUT[i] == transfer)
			{
				libusb->ctrlOUTPending[i] = false;
				break;
			}
		}
	}
}

static void onControlResponse(libusb_transfer *transfer)
{
	USBCommState *state = (USBCommState*)transfer->user_data;
	libusb_state_int *libusb = state->driver_state;
	struct libusb_control_setup *setup = (struct libusb_control_setup*)transfer->buffer;

	if (transfer->status != 0)
	{
		LOG(LUSB, LDebug, "Control IN Req (%hhu, %hu, %hu) returned with status %d!\n", setup->bRequest, setup->wValue, setup->wIndex, (int)transfer->status);
	}

	bool transparentResend = true;
	int res = handleTransferStatus(transfer, asprintf_s("Control IN Transfer (%hhu, %hu, %hu)", setup->bRequest, setup->wValue, setup->wIndex), true, transparentResend);
	bool finalHandling = res != 1 || !transparentResend;

	if (state->onControlResponse != NULL && finalHandling && state->deviceConnected)
		state->onControlResponse(setup->bRequest, libusb_le16_to_cpu(setup->wValue), libusb_le16_to_cpu(setup->wIndex), transfer->buffer+sizeof(struct libusb_control_setup), transfer->actual_length, state->userState, state->userDevice, res == 0);

	if (finalHandling)
	{ // Not already re-sent
		for (int i = 0; i < CTRL_NUM_TRANSFERS; i++)
		{
			if (libusb->controlIN[i] == transfer)
			{
				libusb->ctrlINPending[i] = false;
				break;
			}
		}
	}
}

static void onInterruptIN(libusb_transfer *transfer)
{
	auto receiveTime = sclock::now();
	USBCommState *state = (USBCommState*)transfer->user_data;
	int ep = transfer->endpoint&0xF;
	//LOG(LUSB, LTrace, "|%d:%d\n", ep, transfer->actual_length);

	if (transfer->status != LIBUSB_TRANSFER_COMPLETED)
	{
		handleTransferStatus(transfer, asprintf_s("Interrupt IN Transfer %p for ep %hhu", transfer, transfer->endpoint), false, true);
		return;
	}
	state->driver_state->transferErrors = 0;

	if (state->onUSBPacketIN != NULL && state->commStreaming)
		state->onUSBPacketIN(transfer->buffer, transfer->actual_length, receiveTime, ep, state->userState, state->userDevice);

	if (state->recordStats)
	{
		// Mutex not needed anymore since stats are only accessed in onUSBPacketIN
		//state->statMutex.lock();
		auto &stats = state->streamingEPStats[transfer];
		stats.ep = transfer->endpoint;
		stats.lastReceive = receiveTime;
		stats.receiveCount++;
		stats.receiveBytes += transfer->actual_length;
		state->largestIntLag = std::max(state->largestIntLag, dt(state->lastUSBIntIn, receiveTime));
		state->lastUSBIntIn = receiveTime;
		//state->statMutex.unlock();
	}

	if (state->commStreaming)
		libusb_submit_transfer(transfer);
}

static void usbEventHandler(libusb_context_int *libusb)
{
	struct timeval tv;
	tv.tv_sec = 60;
	tv.tv_usec = 0;
	// Unfortunately rely on timeout to exit.
	// Since context is always active even with no device attached, and hotplug support is not reliable
	// We cannot use the suggestion to "exit with last device" or "exit with deregistering hotplug"
	// So we rely on timeout to be able to check usbEventHandlerRun

#if defined(_WIN32)
#elif defined(__unix__)
	struct rlimit limit;
	getrlimit(RLIMIT_NICE, &limit);
	errno = 0;
	int nc = nice(-40);
	// Expected to fail without sudo privileges
	if (errno)
		LOG(LUSB, LDarn, "Failed to set USB thread nice value to minimum of %d! %s\n", 20-(int)limit.rlim_cur, strerror(errno))
	else
		LOG(LUSB, LDarn, "Set USB thread nice value to %d (minimum of %d)!\n", nc, 20-(int)limit.rlim_cur)
#endif

	while (libusb->usbEventHandlerRun)
	{
		libusb_handle_events_timeout(libusb->context, &tv);
	}
	libusb_exit(libusb->context);
}