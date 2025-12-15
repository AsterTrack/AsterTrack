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

#include "server.hpp"
#include "device/tracking_controller.hpp"
#include "device/tracking_camera.hpp"
#include "device/parsing.hpp"

#include "comm/usb.hpp"

#include "ui/shared.hpp"
#include "util/log.hpp"

// USB Handlers
static void ReadUSBPacket(ServerState &state, TrackingControllerState &controller, uint8_t *data, int length, TimePoint_t receiveTime, uint8_t endpoint);
static void onControlResponse(uint8_t request, uint16_t value, uint16_t index, uint8_t *data, int length, void *userState, std::shared_ptr<void> &userDevice, bool success);
static void onUSBPacketIN(uint8_t *data, int length, TimePoint_t receiveTime, uint8_t endpoint, void *userState, std::shared_ptr<void> &userDevice);
static void checkControlRequest(TrackingControllerState &controller, TrackingControllerState::USBRequest &request, const char* label, USBCommand command, int intervalMS);
static void LogUSBStats(TrackingControllerState &controller, long timeUS);

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

// ----------------------------------------------------------------------------
// Controller Device Setup
// ----------------------------------------------------------------------------

int DetectNewControllers(ServerState &state)
{
	// Setup new controllers
	std::vector<std::shared_ptr<USBCommState>> newControllers = comm_connect(state.libusb_context);
	for (auto &comm : newControllers)
	{
		std::shared_ptr<TrackingControllerState> controller = std::make_shared<TrackingControllerState>();
		comm->userState = &state;
		comm->userDevice = std::static_pointer_cast<void>(controller); // new shared_ptr
		comm->onControlResponse = onControlResponse;
		comm->onUSBPacketIN = onUSBPacketIN;
		controller->comm = std::move(comm);
		controller->endpoints.resize(1);
		controller->id = state.controllers.size();
		// TODO: Make controller IDs persistent, crucial for future persistent setup configurations
		state.controllers.push_back(std::move(controller));
	}

	if (!newControllers.empty())
		LOG(LControllerDevice, LInfo, "%d new controllers connected (%d total)!\n", (int)newControllers.size(), (int)state.controllers.size());
	return newControllers.size();
}

void DisconnectController(ServerState &state, TrackingControllerState &controller)
{
	std::unique_lock dev_lock(state.deviceAccessMutex); // controllers

	// Clean up cameras if they are now unconnected
	for (int c = 0; c < controller.cameras.size(); c++)
	{
		if (!controller.cameras[c])
			continue;
		controller.cameras[c]->controller = nullptr;
		controller.cameras[c]->port = -1;
		CameraCheckDisconnected(state, *controller.cameras[c]);
	}

	// Clean up controller
	comm_disconnect(controller.comm);
	controller.sync = nullptr;
	if (controller.syncGen)
	{ // Delete sync group
		DeleteSyncGroup(*state.stream.contextualLock(), std::move(controller.syncGen));
	}

	// Remove controller
	auto c = std::find_if(state.controllers.begin(), state.controllers.end(), [&controller](const auto &c) { return *c == controller; });
	state.controllers.erase(c);

	// May not have been called if controller had only connecting cameras
	SignalServerEvent(EVT_UPDATE_CAMERAS);

	// Stop streaming if there are no cameras left
	if (state.cameras.empty() && state.isStreaming)
	{
		LOG(LDefault, LWarn, "Leaving streaming mode since all cameras are disconnected after controller disconnect!");
		StopStreaming(state);
	}
}

void HandleController(ServerState &state, TrackingControllerState &controller)
{
	if (controller.comm->commStreaming || state.isStreaming)
	{ // TODO: Send camera disconnect events via interrupt endpoints, too
		// Currently, we rely on getting some rare control requests through to find out a camera disconnected
		// This is quite suboptimal, to be honest. Allow controller to send status packets on it's own when it changed
		checkControlRequest(controller, controller.statusReq, "Status", COMMAND_IN_STATUS, 50);
	}
	else
	{
		checkControlRequest(controller, controller.statusReq, "Status", COMMAND_IN_STATUS, 50);
		checkControlRequest(controller, controller.debugReq, "Debug", COMMAND_IN_DEBUG, 10);
		checkControlRequest(controller, controller.eventReq, "Event", COMMAND_IN_EVENTS, 10);
		checkControlRequest(controller, controller.packetReq, "Packet", COMMAND_IN_PACKETS, 10);
	}

	if (!controller.comm->commStreaming)
		controller.comm->lastUSBStatCheck = sclock::now();

	// Requires mutex in usb transfer if called here, not good
	/* long timeUS = dtUS(controller.comm->lastUSBStatCheck, sclock::now());
	if (timeUS > 2000000)
	{
		controller.comm->lastUSBStatCheck = sclock::now();
		LogUSBStats(controller, timeUS);
	} */
}

void ParseControllerPackets(ServerState &state, TrackingControllerState &controller)
{
	static TimePoint_t last = sclock::now(); // TODO: This is static, so shared between controllers. Makes no sense.
	TimePoint_t start = sclock::now();
	int packetParsed = 0;
	while (true)
	{
		TimePoint_t s0 = sclock::now();
		std::vector<TrackingControllerState::USBPacket> packets;
		{
			auto queue_lock = controller.packetQueue.contextualLock();
			while (!queue_lock->empty())
			{
				packets.push_back(std::move(queue_lock->front()));
				queue_lock->pop();
			}
		}
		if (packets.empty()) break;
		TimePoint_t s1 = sclock::now();
		float accumMS = dtMS(last, s1);
		last = s0;
		for (auto &packet : packets)
		{
			ReadUSBPacket(state, controller, packet.data.data(), packet.data.size(), packet.receiveTime, packet.endpoint);
		}
		packetParsed += packets.size();
		TimePoint_t s2 = sclock::now();
		if (dtUS(s0, s2) > 1000)
		{
			LOG(LParsing, LDarn, "Parsing %d usb packets accumulated over %.2fms took %.2fms of processing plus %ldus of dequeuing!",
				(int)packets.size(), accumMS, dtMS(s1, s2), dtUS(s0,s1));
		}
		if (dtUS(start, s2) > 2000)
		{
			LOG(LParsing, LDarn, "Been parsing %d packets for %.2fms, with %d left in queue, leaving loop!",
				packetParsed, dtMS(start, s2), (int)packets.size());
			break;
		}
	}
}

// ----------------------------------------------------------------------------
// Controller Streaming Setup
// ----------------------------------------------------------------------------

static void SetupSyncGroups(ServerState &state)
{
	auto stream_lock = state.stream.contextualLock();
	// TODO: Setup sync groups in EnsureCamera (based on prior config, e.g. in UI) 2/3 - Remove this

	// Setup sync groups
	std::shared_ptr<Synchronised<SyncGroup>> masterSync;
	for (auto &controller : state.controllers)
	{
		if (!controller->comm->commStreaming) continue;

		// Setup sync group
		controller->sync = nullptr;
		if (masterSync && controller->syncGen)
		{ // Supposed to be synced up with master, not generate own sync
			DeleteSyncGroup(*stream_lock, std::move(controller->syncGen));
			controller->syncGen = nullptr;
		}
		else if (!masterSync && !controller->syncGen)
		{ // No master and no own sync yet, set up to generate sync
			controller->syncGen = std::make_shared<Synchronised<SyncGroup>>();
			stream_lock->syncGroups.push_back(controller->syncGen); // new shared_ptr
			// TODO: Controller might have external sync source, set to SYNC_EXTERNAL then
			auto sync_lock = controller->syncGen->contextualLock();
			sync_lock->source = SYNC_INTERNAL;
			sync_lock->frameIntervalMS = 1000.0f / state.controllerConfig.framerate;
		}
		else if (controller->syncGen)
		{ // Clear existing syncGen
			ClearSyncGroup(*controller->syncGen->contextualLock());
		}
		// TODO: Give control which one generates signal, but not 100% necessary
		// TODO: Allow for multiple syncGroups (so multiple controllers with syncGen)
		if (controller->syncGen)
			masterSync = controller->syncGen; // new shared_ptr

		// Select sync source for controller
		controller->sync = masterSync? masterSync : controller->syncGen;

		// Enter cameras into sync group
		for (int c = 0; c < controller->cameras.size(); c++)
		{
			if (controller->cameras[c])
			{
				SetCameraSync(*stream_lock, controller->cameras[c], controller->sync);
			}
		}
	}

	// Reset sync group states
	ResetStreamState(*stream_lock);

	for (auto &cam : state.cameras)
	{
		if (cam->sync)
		{
			// TODO: Implement control over FPS in free-running cameras
			// also see other TODO on this (same in DeviceUpdateCameraSetup)
			// E.g. OV9281 drivers cannot set framerate yet
			// So its always running at 144Hz for 1280x800 and 253Hz for 640x400
			auto sync_lock = cam->sync->contextualLock();
			if (sync_lock->source == SYNC_NONE)
				sync_lock->frameIntervalMS = 1000.0f / 144;
		}
	}
}

static void SetupVirtualSyncGroup(ServerState &state)
{
	if (!state.controllers.empty()) return;
	auto stream_lock = state.stream.contextualLock();
	if (!stream_lock->syncGroups.empty()) return;
	// Setup virtual sync group
	stream_lock->syncGroups.push_back(std::make_shared<Synchronised<SyncGroup>>()); // new shared_ptr
	auto sync_lock = stream_lock->syncGroups.back()->contextualLock();
	ResetSyncGroup(*sync_lock);
	sync_lock->source = SYNC_VIRTUAL;
	sync_lock->frameIntervalMS = 1000.0f / state.controllerConfig.framerate;
	// Start with a frame that will be continued
	SyncedFrame frame = {};
	frame.cameras.resize(sync_lock->cameras.size());
	frame.ID = 0;
	frame.approxSOF = false;
	frame.SOF = sclock::now();
	LOG(LStreaming, LInfo, "Started virtual frame generation!\n");
	sync_lock->frames.push_back(std::move(frame));
	sync_lock->frameCount++;
}

static void DeleteVirtualSyncGroup(ServerState &state)
{
	auto stream_lock = state.stream.contextualLock();
	if (stream_lock->syncGroups.empty()) return;
	for (auto it = stream_lock->syncGroups.begin(); it != stream_lock->syncGroups.end();)
	{
		if (it->get()->contextualRLock()->source == SYNC_VIRTUAL)
			it = stream_lock->syncGroups.erase(it);
		else it++;
	}
}

bool ControllerUpdateSyncMask(TrackingControllerState &controller)
{
	if (!controller.comm->commStreaming) return false;
	if (!controller.sync || controller.sync->contextualRLock()->source == SYNC_NONE) return false;

	// Select cameras that have been setup and chosen for streaming
	uint16_t portMask = 0;
	for (auto &camera : controller.cameras)
	{
		if (camera && camera->isStreaming())
			portMask |= 1 << camera->port;
	}
	comm_submit_control_data(controller.comm, COMMAND_OUT_SYNC_MASK, 0x00, portMask);
	return true;
}

void DevicesStartStreaming(ServerState &state)
{
	std::shared_lock dev_lock(state.deviceAccessMutex);

	for (auto &controller : state.controllers)
	{
		ResetTimeSync(*controller->timeSync.contextualLock());

		// Open the comm channels (interrupt transfers) to the controller for streaming
		if (!comm_startStream(controller->comm))
		{
			LOG(LControllerDevice, LError, "Failed to open comm channels to controller %d!\n", controller->id);
		}
		else
		{
			LOG(LControllerDevice, LInfo, "Opened comm channels to controller %d!\n", controller->id);
			controller->endpoints.resize(controller->comm->streamingEndpoints.load()+1);
			for (auto &ep : controller->endpoints)
				ep.lastReceived = sclock::now();
		}
	}

	// Give controller some time to initialise, not strictly needed
	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	for (auto &controller : state.controllers)
	{
		if (!controller->comm->commStreaming) continue;

		// Request to start establishing a solid timesync with both the host (this) and the cameras
		if (comm_submit_control_data(controller->comm, COMMAND_OUT_TIME_SYNC, true, 0) < 0)
		{
			LOG(LControllerDevice, LError, "Failed to send time sync request to controller %d!\n", controller->id);
		}
		else
		{

			LOG(LControllerDevice, LInfo, "Requesting to start time sync with controller %d!\n", controller->id);
		}

		for (auto &ep : controller->endpoints)
			ep.lastReceived = sclock::now();
	}

	// Give controllers some time to establish a time sync, not strictly needed
	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	for (auto &cam : state.cameras)
	{
		if (!cam->client && (!cam->controller || !cam->controller->comm->commStreaming
			|| cam->state.contextualRLock()->commState != CommPiReady))
			continue;

		// Send setup data incase it didn't already happen
		CameraUpdateSetup(state, *cam);

		// Make sure secondary features match expected state
		CameraUpdateStream(*cam);
		CameraUpdateVis(*cam);
	}

	// Give cameras some time to configure themselves
	std::this_thread::sleep_for(std::chrono::milliseconds(50));

	for (auto &cam : state.cameras)
	{
		if (!cam->client && (!cam->controller || !cam->controller->comm->commStreaming)) continue;

		// Communicate with camera to start streaming (startup time is high)
		cam->sendModeSet(TRCAM_FLAG_STREAMING | TRCAM_MODE_BLOB, false);
	}
	TimePoint_t modeSetTime = sclock::now();
	TimePoint_t modeSetTimeout = sclock::now() + std::chrono::milliseconds(500);

	// Configure sync groups
	SetupSyncGroups(state);

	// Set external sync source for relevant controllers
	for (auto &controller : state.controllers)
	{
		if (!controller->comm->commStreaming) continue;

		if (controller->syncGen && controller->syncGen->contextualRLock()->source == SYNC_INTERNAL)
			continue; // Start generating last
		
		if (controller->sync && controller->sync->contextualRLock()->source != SYNC_NONE)
		{ // Request to copy external sync input
			comm_submit_control_data(controller->comm, COMMAND_OUT_SYNC_EXTERNAL, 0, 0);
		}
		else
		{ // Request to not do any sync - cameras will need to be free-running
			comm_submit_control_data(controller->comm, COMMAND_OUT_SYNC_RESET, 0, 0);
		}
	}

	LOG(LGUI, LInfo, "Setup cameras and sync groups");

	for (auto &cam : state.cameras)
	{ // Wait for cameras to send mode change packets to signal they are ready to receive frame syncs
		while (sclock::now() < modeSetTimeout && cam->mode != cam->modeSet.mode)
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
	for (auto &cam : state.cameras)
	{ // Set handleIndividually flag right before we update controller with streaming status for each camera port
		// If this cameras mode hasn't changed yet, it will have to finish configuring it's streaming state later
		cam->modeSet.handleIndividually = true;
	}

	LOG(LGUI, LInfo, "Waited %fms for cameras to change modes!", dtMS(modeSetTime, sclock::now()));

	// Setup sync masks for all controllers
	for (auto &controller : state.controllers)
	{
		if (!ControllerUpdateSyncMask(*controller))
		{
			LOG(LGUI, LInfo, "Failed to setup controller %d for streaming!", controller->id);
		}
	}

	// Set generating sync source for relevant controllers
	for (auto &controller : state.controllers)
	{
		if (!controller->comm->commStreaming) continue;
		if (!controller->syncGen) continue;
		// Request to start generating frame signals
		auto sync_lock = controller->syncGen->contextualRLock();
		if (sync_lock->source == SYNC_INTERNAL)
			comm_submit_control_data(controller->comm, COMMAND_OUT_SYNC_GENERATE, (uint16_t)1000.0f/sync_lock->frameIntervalMS, 0);
	}
	LOG(LGUI, LInfo, "Started camera sync");

	// In case no controller is connected, provide virtual sync group for e.g. IMUs
	SetupVirtualSyncGroup(state);
}

void DevicesStopStreaming(ServerState &state)
{
	std::shared_lock dev_lock(state.deviceAccessMutex);

	for (auto &cam : state.cameras)
	{ // Communicate with cameras to stop streaming
		cam->sendModeSet(TRCAM_STANDBY, false);
	}
	TimePoint_t modeSetTimeout = sclock::now() + std::chrono::milliseconds(100);

	for (auto &cam : state.cameras)
	{ // Wait for cameras to send mode change packets to signal they are ready to receive frame syncs
		while (sclock::now() < modeSetTimeout && cam->mode != cam->modeSet.mode)
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
	for (auto &cam : state.cameras)
	{ // Set handleIndividually flag right before we update controller with streaming status for each camera port
		// If this cameras mode hasn't changed yet, it will have to finish configuring it's streaming state later
		cam->modeSet.handleIndividually = false;
	}

	// Communicate with controllers to prepare to stop streaming
	for (auto &controller : state.controllers)
	{
		// Request to stop triggering new frames
		comm_submit_control_data(controller->comm, COMMAND_OUT_SYNC_RESET, 0x00, 0);
		// Request to disable time sync
		comm_submit_control_data(controller->comm, COMMAND_OUT_TIME_SYNC, false, 0);
	}

	// Delay a bit to give last packets a chance to be sent
	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	// Communicate with controllers to close comm channels
	for (auto &controller : state.controllers)
	{
		// Stop receiving transfers from controller
		comm_stopStream(controller->comm);
		controller->endpoints.clear();
		controller->endpoints.resize(1);

		for (auto &port : controller->ports)
			ResetPacketPort(port);
	}

	// Sync Group not attached to any controller - remove
	DeleteVirtualSyncGroup(state);

	// Reset streaming states
	ResetStreamState(*state.stream.contextualLock());

	// Reset camera device state
	for (auto &cam : state.cameras)
		cam->receiving = {};
}

// ----------------------------------------------------------------------------
// Controller USB Handlers
// ----------------------------------------------------------------------------

static void ReadUSBPacket(ServerState &state, TrackingControllerState &controller, uint8_t *data, int length, TimePoint_t receiveTime, uint8_t endpoint)
{
	struct PacketState
	{
		ServerState *state;
		TrackingControllerState *controller;
		TimePoint_t receiveTime;
		int endpoint;
	} packetState = { &state, &controller, receiveTime, endpoint };

	// TODO: This is not properly synchronised/secured, deviceAccessMutex is not even locked
	// Consider at least copying shared_ptr of camera

	// Each USB frame contains multiple blocks of data from multiple ports
	// Each block belongs to exactly one packet from any port
	// This function extracts the blocks and executes the callbacks
	// Importantly, it also makes sure the blocks are processed in the correct order
	// It does this using a continuous blockID and storing out-of-order blocks
	// But for low latency, it can parse packets in parallel, so that a delayed block
	// Does not hinder parsing of blocks of newer packets
	HandlePacketBlocks(
		controller.ports, data, length, &packetState,
		[](void *userData, SignalTag signal, uint8_t *data, uint16_t length) { // onSignal
			PacketState &packetState = *((PacketState*)userData);
			if (signal == SIGNAL_INVALID)
			{
				LOG(LParsing, LTrace, "Skipped invalid block of length %d on endpoint %d\n", length, packetState.endpoint);
			}
			else if (signal == SIGNAL_SOF)
			{
				ReadSOFPacket(*packetState.controller, data, length, packetState.receiveTime);
			}
			else if (signal == SIGNAL_DEBUG)
			{
				ReadDebugPacket(*packetState.controller, data, length);
			}
			else if (signal == SIGNAL_EVENT)
			{
				ReadEventPacket(*packetState.controller, data, length);
			}
		},
		[](void *userData, int port, PacketBlocks &packet) { // onPacketHeader
			PacketState &packetState = *((PacketState*)userData);
			std::shared_ptr<TrackingCameraState> &camera = packetState.controller->cameras[port];
			if (!camera)
			{
				packet.ignored = true;
				LOG(LStreaming, LError, "---- Packet from camera on port %d that has not been set up yet!\n", port);
				return;
			}
			if (!camera->sync && (packet.header.tag == PACKET_FRAME_SIGNAL || packet.header.isStreamPacket()))
			{
				packet.ignored = true;
				LOG(LStreaming, LTrace, "Camera %d (Port %d) sent a streaming packet but was not set up for streaming!\n", camera->id, port);
				return;
			}

			if (packet.header.tag == PACKET_FRAME_SIGNAL)
			{ // Frame is starting to be processed
				auto sync_lock = camera->sync->contextualLock();
				SyncedFrame *frame = RegisterCameraFrame(*sync_lock, camera->syncIndex, packet.header.frameID);
				if (!frame)
				{
					packet.ignored = true;
				}
				else
				{
					LOG(LStreaming, LTrace, "Camera %d (Port %d) announced packet for frame %d (%d)!\n", camera->id, port, frame->ID, packet.header.frameID&0xFF);
				}
			}
			else if (packet.header.isStreamPacket())
			{ // Register that camera is receiving frame data
				if (!RegisterStreamPacket(*camera->sync->contextualLock(), camera->syncIndex, packet.header.frameID, packetState.receiveTime))
					packet.ignored = true;
			}
		},
		[](void *userData, int port, PacketBlocks &packet, uint8_t *data, int length) { // onPacketBlock
			if (packet.ignored) return;
			PacketState &packetState = *((PacketState*)userData);
			std::shared_ptr<TrackingCameraState> &camera = packetState.controller->cameras[port];
			if (!camera)
			{
				packet.ignored = true;
				LOG(LStreaming, LError, "---- Camera on port %d got removed since packet was first received!\n", port);
				return;
			}

			if (packet.header.isStreamPacket())
			{ // Update statistics of streaming packet
				if (!RegisterStreamBlock(*camera->sync->contextualLock(), camera->syncIndex, packet.header.frameID))
					packet.ignored = true;
			}
		},
		[](void *userData, int port, PacketBlocks &packet) { // onPacketDone
			if (packet.ignored) return;
			PacketState &packetState = *((PacketState*)userData);
			std::shared_ptr<TrackingCameraState> &camera = packetState.controller->cameras[port];
			if (!camera)
			{
				packet.ignored = true;
				LOG(LStreaming, LError, "---- Camera on port %d got removed since packet was first received!\n", port);
				return;
			}

			if (packet.erroneous)
			{
				LOG(LStreaming, LError, "Camera %d had packet %d discarded because of one or more long missing blocks after header %d!\n",
					camera->id, packet.header.tag, packet.headerBlockID);
				if (packet.header.isStreamPacket())
				{
					RegisterStreamPacketComplete(*camera->sync->contextualLock(), camera->syncIndex, packet.header.frameID, {}, true);
				}
				return;
			}
			if (!VerifyChecksum(packet))
				packet.erroneous = true;
			if (packet.header.isStreamPacket())
			{
				auto cameraFrame = ReadStreamingPacket(*camera, packet);
				int blobCount = cameraFrame.rawPoints2D.size();
				auto sync_lock = camera->sync->contextualLock();
				SyncedFrame *frame = RegisterStreamPacketComplete(*sync_lock, camera->syncIndex, packet.header.frameID, std::move(cameraFrame), packet.erroneous);
				if (frame)
				{ // Packet existed for frame
					LOG(LStreaming, LTrace, "Camera %d (Port %d) fully transmitted stream packet %d for frame %d (%d) with %d blobs!\n",
						camera->id, port, packet.header.tag, frame->ID, frame->ID&0xFF, blobCount);
					if (frame->previouslyProcessed)
					{
						LOG(LStreaming, LTrace, "---- Camera %d finally transmitted stream packet for frame %d with %d blobs, %.2fms into the frame, %.2fms after processing!\n",
							camera->id, frame->ID, blobCount, dtMS(frame->SOF, sclock::now()), dtMS(frame->lastProcessed, sclock::now()));
					}
				}
				else
				{
					LOG(LParsing, LError, "---- Completed stream packet for non-existant frame %d or unregistered stream packet!\n", packet.header.frameID);
				}
			}
			else
			{
				ReadCameraPacket(*camera, packet);
			}
		});
}

static void onControlResponse(uint8_t request, uint16_t value, uint16_t index, uint8_t *data,
	int length, void *userState, std::shared_ptr<void> &userDevice, bool success)
{
	// TODO: Do shared_ptr to ensure it's still here - it may not be
	TrackingControllerState &controller = *static_cast<TrackingControllerState*>(userDevice.get());
	ServerState &state = *(ServerState *)userState;
	auto clearControlRequest = [&](TrackingControllerState::USBRequest &request, const char* label)
	{
		if (request.stalling)
		{
			float deltaT = dtMS(request.submitted, sclock::now());
			if (success)
			{
				LOG(LUSB, LWarn, "Stalled control transfer %d (%s) completed after %fms",
					request.transfer.load(), label, deltaT);
			}
			else
			{ // Libusb timeout
				LOG(LUSB, LWarn, "Stalled control transfer %d (%s) got cancelled after %fms",
					request.transfer.load(), label, deltaT);
			}
			request.stalling = false;
		}
		request.transfer = -1;
	};
	switch (request)
	{
	case COMMAND_IN_STATUS:
		clearControlRequest(controller.statusReq, "Status");
		break;
	case COMMAND_IN_DEBUG:
		clearControlRequest(controller.debugReq, "Debug");
		break;
	case COMMAND_IN_EVENTS:
		clearControlRequest(controller.eventReq, "Event");
		break;
	case COMMAND_IN_PACKETS:
		clearControlRequest(controller.packetReq, "Packet");
		break;
	default:
		break;
	}
	if (!success)
		return;

	switch (request)
	{
	case COMMAND_IN_STATUS:
	{ // Camera Status Response
		ReadStatusPacket(state, controller, data, length);
		break;
	}
	case COMMAND_IN_DEBUG:
	{ // Debug Response
		ReadDebugPacket(controller, data, length);
		break;
	}
	case COMMAND_IN_EVENTS:
	{ // Event Response
		if (state.isStreaming)
		{ // TimeSync is only accurate when streaming anyways
			ReadEventPacket(controller, data, length);
		}
		break;
	}
	case COMMAND_IN_PACKETS:
	{ // Packet Response
		if (length <= USB_PACKET_HEADER)
			break;
		if (state.usePacketQueue)
		{ // Queue packet instead of handling it here to keep USB thread clear
			auto queue_lock = controller.packetQueue.contextualLock();
			queue_lock->emplace(sclock::now(), 0, std::vector<uint8_t>(data+USB_PACKET_HEADER, data+length));
		}
		else
			ReadUSBPacket(state, controller, data+USB_PACKET_HEADER, length-USB_PACKET_HEADER, sclock::now(), 0);
		break;
	}
	default:
	{
		std::stringstream hexBuf;
		printBuffer(hexBuf, data, length);
		LOG(LControllerDevice, LWarn, "Unknown Control Response %d (%d): %s\n", request, length, hexBuf.str().c_str());
		break;
	}
	}
}

static void onUSBPacketIN(uint8_t *data, int length, TimePoint_t receiveTime, uint8_t endpoint, void *userState, std::shared_ptr<void> &userDevice)
{
	TrackingControllerState &controller = *static_cast<TrackingControllerState*>(userDevice.get());
	ServerState &state = *((ServerState*)userState);
	if (state.mode != MODE_Device || !state.isStreaming)
	{ // Not expecting any transfer
		LOG(LUSB, LDebug, "Unexpected transfer IN while not streaming!\n");
		return;
	}
	if (!controller.comm || !controller.comm->deviceConnected)
	{
		LOG(LUSB, LError, "Unexpected transfer after device was disconnected!\n");
		return;
	}
	if (endpoint >= controller.endpoints.size())
	{
		LOG(LUSB, LError, "Unexpected transfer on endpoint %d! Only set up %d\n", endpoint, (int)controller.endpoints.size());
		return;
	}

	// No need to lock state.deviceAccessMutex, TrackingControllerState is guaranteed to still exist, before destruction comm is cleaned up properly
	// Make sure cameras are only destroyed well after any transfers might reference them - or copy shared_ptr when accessing them here
	//std::shared_lock dev_lock(state.deviceAccessMutex);
	// TODO: Properly replace all those random mutexes (deviceAccess, pipeline) with Synchronised constructs where necessary
	// Or find another construct that efficiently and easily enforces and protects these relevant states
	// Currently, there's no major problem, but it's unsustainable

	{ // Read packet header
		USBPacketHeader header = parseUSBPacketHeader(data);
		// Make sure packet is continuous within endpoint
		auto &stats = controller.endpoints[endpoint]; // Assumes all endpoints are continuous, e.g. ep 1, 2, 3, 4
		if (header.counter != (stats.counter+1)%256)
		{ // Controller must have thought it send a packet but didn't - this really shouldn't happen
			LOG(LUSB, LError, "Dropped packet in endpoint %d! Skipped from %d to %d", endpoint, stats.counter, header.counter);
		}
		else
		{ // Is continuous, got timestamp that the last packet within the endpoint was sent at for use with time sync
			float commLag = dtMS(stats.lastReceived, sclock::now());
			if (commLag > 20)
			{ // Still fine, but controller should be responsible for sending a packet every ms at least to keep up time sync
				LOG(LTimesync, LWarn, "Controller comms dropped out for %fms, this is bad for time sync!", commLag);
			}
			else if (commLag > 5)
			{ // Still fine, but controller should be responsible for sending a packet every ms at least to keep up time sync
				LOG(LTimesync, LDarn, "Controller comms dropped out for %fms, this is suboptimal for time sync!", commLag);
			}
			// Update time sync with the last packet in the endpoint as the newly gathered sample point
			auto sync_lock = controller.timeSync.contextualLock();
			std::pair<uint64_t, TimePoint_t> packetSentSync = UpdateTimeSync(*sync_lock, header.lastTimestamp, 1<<24, stats.lastReceived);
			LOG(LTimesync, LTrace, "Packet of size %d had timestamp %uus for last packet received %.2fms ago, estimated to be sent %.2fms ago (this packet was received %.2fms ago)\n",
				length, header.lastTimestamp, dtMS(stats.lastReceived, sclock::now()), dtMS(packetSentSync.second, sclock::now()), dtMS(receiveTime, sclock::now()));

			if (packetSentSync.first > 0 && controller.timingRecord.recordTimeSync)
				controller.timingRecord.timeSync.push_back({ packetSentSync.first, packetSentSync.second, stats.lastReceived });
		}
		stats = { header.counter, receiveTime };

		data += USB_PACKET_HEADER;
		length -= USB_PACKET_HEADER;
	}

	if (length == 0)
	{ // Null-packets can be sent to enforce timesync when nothing else needs to be sent
		return;
	}

	if (state.usePacketQueue)
	{ // Queue packet instead of handling it here to keep USB thread clear
		auto queue_lock = controller.packetQueue.contextualLock();
		queue_lock->emplace(receiveTime, endpoint, std::vector<uint8_t>(data, data+length));
		state.parsing_cv.notify_one();
	}
	else
	{
		ReadUSBPacket(state, controller, data, length, receiveTime, endpoint);
		MaintainStreamState(*state.stream.contextualLock());
	}

	long timeUS = dtUS(controller.comm->lastUSBStatCheck, sclock::now());
	if (timeUS > 2000000)
	{
		controller.comm->lastUSBStatCheck = sclock::now();
		if (timeUS < 3000000)
			LogUSBStats(controller, timeUS);
	}
}

static void checkControlRequest(TrackingControllerState &controller, TrackingControllerState::USBRequest &request, const char* label, USBCommand command, int intervalMS)
{
	float deltaT = dtMS(request.submitted, sclock::now());
	if (request.transfer < 0 && deltaT > intervalMS)
	{ // Interval lapsed, request transfer
		request.submitted = sclock::now();
		request.transfer = comm_submit_control_request(controller.comm, command, 0, 0);
		if (request.transfer < 0)
		{
			LOG(LUSB, LWarn, "Failed to request %s control transfer!\n", label);
		}
		else
		{
			LOG(LUSB, LTrace, "Allocated control transfer %d to %s request\n",
				request.transfer.load(), label);
		}
	}
	else if (request.transfer >= 0 && deltaT > 20)
	{ // Just waiting longer since it probably indicates an error with the controller and we don't want to spam the log
		// But libusb will timeout the transfer before anyway
		LOG(LUSB, LDarn, "Controller didn't respond to control transfer %d (%s) in %fms, cancelling!\n",
			request.transfer.load(), label, deltaT);
		comm_cancel_control_request(controller.comm, request.transfer);
		request.transfer = -1;
		request.stalling = false;
	}
	else if (request.transfer >= 0 && deltaT > 10 && !request.stalling)
	{ // Over 10ms to answer transfer is very odd
		LOG(LUSB, LDarn, "Controller didn't respond to control transfer %d (%s) in %fms!\n",
			request.transfer.load(), label, deltaT);
		request.stalling = true;
	}
};

static void LogUSBStats(TrackingControllerState &controller, long timeUS)
{
	// Called from the same thread, so fine without mutex
	//controller.comm->statMutex.lock();
	std::map<void*, TransferStats> stats;
	controller.comm->streamingEPStats.swap(stats);
	float largestLag = controller.comm->largestIntLag;
	controller.comm->largestIntLag = 0;
	//controller.comm->statMutex.unlock();

	std::map<int, std::tuple<int, int, int>> endpoints;
	for (auto &stat : stats)
	{
		auto &ep = endpoints[stat.second.ep];
		std::get<0>(ep)++;
		std::get<1>(ep) += stat.second.receiveCount;
		std::get<2>(ep) += stat.second.receiveBytes;
	}
	LOG(LUSB, endpoints.empty()? LTrace : LDebug, "%d eps received a transfer over the past %.2fs, with the largest combined lag of %.2fms\n",
		(int)endpoints.size(), timeUS/1000000.0f, largestLag);
	for (auto &ep : endpoints)
	{
		#define INT_TRANSFER_SIZE 1024 // From usb.cpp
		LOG(LUSB, LDebug, "    EP %d: %d queues with %d transfers of %d bytes total, %.0f%% fill rate, %.0f%% throughput, %.0f%% transfer rate\n",
			ep.first, std::get<0>(ep.second), std::get<1>(ep.second), std::get<2>(ep.second),
			(float)std::get<2>(ep.second)/std::get<1>(ep.second)/INT_TRANSFER_SIZE*100,
			std::get<2>(ep.second)/(INT_TRANSFER_SIZE*8*timeUS/1000.0f)*100,
			std::get<1>(ep.second)/(8*timeUS/1000.0f)*100);
		for (auto &stat : stats)
		{
			if (stat.second.ep != ep.first) continue;
			LOG(LUSB, LDebug, "        Queue %p: %d transfers of %d bytes total, %.0f%% fill rate\n",
				stat.first, stat.second.receiveCount, stat.second.receiveBytes,
				(float)stat.second.receiveBytes/stat.second.receiveCount/INT_TRANSFER_SIZE*100);
		}
	}
}