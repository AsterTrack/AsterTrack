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
#include "server_internal.hpp"
#include "pipeline/pipeline.hpp"

#include "device/tracking_controller.hpp"
#include "device/tracking_camera.hpp"
#include "device/wireless_cameras.hpp"
#include "device/parsing.hpp"
#include "comm/usb.hpp"

#include "ui/shared.hpp" // Signals to UI

#include "util/log.hpp"
#include "util/util.hpp" // printBuffer, TimePoint_t
#include "util/threading.hpp"

#include "hidapi/hidapi.h"

#include "ctpl/ctpl.hpp"
extern ctpl::thread_pool threadPool;

#include <chrono>
#include <memory>

// Coprocessing thread
static void DeviceSupervisorThread(std::stop_token stop_token, ServerState *statePtr);

// ----------------------------------------------------------------------------
// Device Mode
// ----------------------------------------------------------------------------

void StartDeviceMode(ServerState &state)
{
	if (state.mode != MODE_None)
		return;

	// Initialise state
	state.mode = MODE_Device;
	state.pipeline.isSimulationMode = false;
	state.pipeline.keepInternalData = false;

	// Connect to IMU providers
	hid_init();
	{ // Fetch IMU providers
		auto imu_lock = state.imuProviders.contextualLock();
		std::unique_lock hid_lock(state.hid_access);
		detectAsterTrackReceivers(*imu_lock);
		detectSlimeVRReceivers(*imu_lock);
		initialiseRemoteIMUs(*imu_lock);
		// TODO: Add more IMU integrations here
	}

	// Connect new controllers
	DetectNewControllers(state);

	// Start server for wireless cameras
	StartWirelessServer(state.server, state);

	// Start device supervisor thread
	assert(state.coprocessingThread == NULL);
	state.coprocessingThread = new std::jthread(DeviceSupervisorThread, &state);

	IntegrationsInit(state.io, state.config);

	SignalServerEvent(EVT_MODE_DEVICE_START);
	SignalServerEvent(EVT_UPDATE_CAMERAS);
}

void StopDeviceMode(ServerState &state)
{
	if (state.mode != MODE_Device)
		return;

	LOG(LDefault, LInfo, "Disconnecting!\n");

	StopStreaming(state);

	StopCoprocessingThread(state);

	IntegrationsCleanup(state.io, state.config);

	std::scoped_lock dev_lock(state.deviceAccessMutex, state.pipeline.pipelineLock);
	state.mode = MODE_None;

	// Disconnect from controllers and their wired-only cameras
	while (!state.controllers.empty())
		DisconnectController(state, *state.controllers.back());

	// Stop wireless server
	StopWirelessServer(state.server);

	// Disconnect from all remaining wireless cameras
	while (!state.cameras.empty())
		CameraCheckDisconnected(state, *state.cameras.back());

	// Diconnect IMU providers
	state.imuProviders.contextualLock()->clear();
	hid_exit();

	assert(state.controllers.empty());
	assert(state.cameras.empty());

	// Reset state
	ResetPipelineState(state.pipeline);
	for (auto &tracker : state.trackerConfigs)
		tracker.imu = nullptr;

	SignalServerEvent(EVT_MODE_DEVICE_STOP);
	SignalServerEvent(EVT_UPDATE_CAMERAS);
}

static void DeviceSupervisorThread(std::stop_token stop_token, ServerState *statePtr)
{
	ServerState &state = *statePtr;

	SetCurrentThreadName("Device Supervisor");

	int it = 0;

	static bool checkingIMU = false;
	TimePoint_t lastContCheck = sclock::now();
	TimePoint_t lastIMUCheck = sclock::now();
	TimePoint_t lastIOCheck = sclock::now();

	while (!stop_token.stop_requested())
	{
		it++;

		TimePoint_t now = sclock::now();

		// Check for disconnected hardware without upgrading a shared_lock deviceAccessMutex to unique_lock
		// The UI thread can do this, and does so a lot, so it should be the only thread allowed to do that
		for (int c = 0; c < state.controllers.size(); c++)
		{
			TrackingControllerState &controller = *state.controllers[c];
			if (controller.comm->deviceConnected)
				continue;

			LOG(LControllerDevice, LWarn, "Communication link of controller died!\n");
			// TODO: Disconnect controller asynchronously somehow
			// Why? Does this take long? Not sure anymore, need to check
			DisconnectController(state, controller);
			c--;
		}
		for (int c = 0; c < state.cameras.size(); c++)
		{
			TrackingCameraState &camera = *state.cameras[c];
			CameraID id = camera.id;
			if (CameraCheckDisconnected(state, camera))
			{
				LOG(LCameraDevice, LWarn, "Camera %u had no remaining communication links!\n", id);
				c--;
				continue;
			}
		}

		// Try to lock, and if failing to do so, yield to (potentially) another thread stopping device mode
		std::shared_lock dev_lock(state.deviceAccessMutex, std::chrono::milliseconds(10));
		if (!dev_lock.owns_lock())
			LOG(LControllerDevice, LError, "Failed to lock deviceAccessMutex in device thread, likely avoided deadlock!");
		if (stop_token.stop_requested()) break;
		if (!dev_lock.owns_lock()) continue;

#if !defined(_WIN32)
		// Does work, but takes over a second on windows because no hotplugging support
		if (dtMS(lastContCheck, now) > 100)
		{ // Detect check for new controllers
			//DetectNewControllers(*state);
			lastContCheck = now;
		}
#endif

		// Handle comms of controllers
		for (int c = 0; c < state.controllers.size(); c++)
		{
			TrackingControllerState &controller = *state.controllers[c];
			if (controller.comm->deviceConnected)
				HandleController(state, controller);
		}

		// Handle comms of cameras
		for (int c = 0; c < state.cameras.size(); c++)
		{
			TrackingCameraState &camera = *state.cameras[c];
			CameraID id = camera.id;
			if (!camera.storage.receivedInfo && camera.hasComms() && dtMS(camera.storage.lastFetchTime, sclock::now()) > 500)
			{
				LOG(LCameraDevice, LDebug, "Requesting info from camera #%u", camera.id);
				camera.storage.lastFetchTime = sclock::now();
				camera.sendPacket(PACKET_CAMERA_INFO, nullptr, 0);
			}
		}

		if (dtMS(lastIOCheck, now) > 10)
		{ // Check Tracking IO for low-priority updates
			lastIOCheck = now;
			IntegrationsUpdate(state.io, state);
		}

		if (!checkingIMU && dtMS(lastIMUCheck, now) > 100)
		{ // Check for new IMU providers
			lastIMUCheck = now;
			checkingIMU = true;
			threadPool.push([](int){
				auto &state = GetState();
				auto providers = *state.imuProviders.contextualLock();
				{ // Fetch new IMU providers
					std::unique_lock hid_lock(state.hid_access);
					detectAsterTrackReceivers(providers);
					detectSlimeVRReceivers(providers);
					initialiseRemoteIMUs(providers);
					// TODO: Add more IMU integrations here
				}
				*state.imuProviders.contextualLock() = providers;
				checkingIMU = false;
			});
		}

		// Fetch IO packets for e.g. Remote IMUs
		IntegrationsReceive(state.io, state);

		bool imusRegistered = false;
		int updatedIMUs = 0;
		IMUDeviceList removedIMUs, addedIMUs;
		{ // Fetch new packets from IMUs
			auto imuLock = state.imuProviders.contextualLock();
			for (auto imuProvider = imuLock->begin(); imuProvider != imuLock->end();)
			{
				auto status = imuProvider->get()->poll(updatedIMUs, removedIMUs, addedIMUs);
				if (status == IMU_STATUS_DISCONNECTED)
				{
					imuProvider = imuLock->erase(imuProvider);
					continue;
				}
				if (status == IMU_STATUS_DEVICES_CONNECTED)
					imusRegistered = true;
				imuProvider++;
			}
		}
		if (!addedIMUs.empty() || !removedIMUs.empty())
		{ // Asynchronously register new IMUs
			threadPool.push([](int, IMUDeviceList &addedIMUs, IMUDeviceList &removedIMUs){
				ServerState &state = GetState();
				// In threadPool so we're not blocking (waiting for frame processing) in device supervisor thread
				std::unique_lock pipeline_lock(state.pipeline.pipelineLock);
				for (auto &imuDevice : addedIMUs)
				{
					if (!imuDevice || imuDevice->index >= 0) continue;
					// TODO: Handle receiver replugging, should probably detect IMU as the same
					// Either replace existing (Probably move old samples in to new IMU?)
					// Or just add new and replace any tracking references
					//auto ex = std::find_if(state.pipeline.record.imus.begin(), state.pipeline.record.imus.end(),
					//	[&](auto &i){ return i->id == imu->id; });
					//if (ex != state.pipeline.record.imus.end())
					// Else just add to imu record
					imuDevice->index = state.pipeline.record.imus.size();
					auto imu = std::static_pointer_cast<IMU>(imuDevice);
					state.pipeline.record.imus.push_back(imu); // new shared_ptr
					auto trackerConfig = std::find_if(state.trackerConfigs.begin(), state.trackerConfigs.end(),
						[&](auto &cfg){ return cfg.imuIdent == imu->id; });
					if (trackerConfig != state.trackerConfigs.end())
					{
						if (state.isStreaming)
							AssociateIMU(state.pipeline, imu, trackerConfig->id, trackerConfig->imuCalib);
						trackerConfig->imu = imu;
					}
					else if (state.isStreaming)
					{
						OrphanIMU(state.pipeline, imu); // new shared_ptr
						LOG(LTracking, LInfo, "Added IMU as orphaned tracked IMU!");
					}
				}
				for (auto &imu : removedIMUs)
				{
					// TODO: Handle removal of IMUs - currently just ignored, and trackers using them will just not get any updates, which is fine
				}
				SignalPipelineUpdate();
			}, std::move(addedIMUs), std::move(removedIMUs));
		}

		if (state.usePacketQueue)
		{
			for (int c = 0; c < state.controllers.size(); c++)
			{ // Parse new packets from controllers
				TrackingControllerState &controller = *state.controllers[c];
				if (controller.comm->deviceConnected)
					ParseControllerPackets(state, controller);
			}
		}

		if (state.isStreaming)
		{ // Frame consistency supervision and processing stream frames once deemed complete
			auto stream_lock = state.stream.contextualLock();
			bool startedProcessing = false;
			if (state.usePacketQueue || dtMS(stream_lock->lastMaintainTime, now) > 5)
			{ // Maintain stream state only if we've not already called MaintainStreamState with a recent USB packet
				startedProcessing = MaintainStreamState(*stream_lock);
			}
			if (!startedProcessing && state.lowLatencyIMU && updatedIMUs > 0)
			{ // Notify realtime processing thread of new IMU samples at least
				state.processing_cv.notify_one();
			}
		}

		if (!state.isStreaming)
		{ // Update status of background processes even when not streaming (anymore)
			UpdatePipelineStatus(state.pipeline);
		}

		// Unlock shared deviceAccessMutex
		dev_lock.unlock();

		// Interval only affects IMU polling rate, not USB packets by controllers
		int interval = imusRegistered? (state.lowLatencyIMU? 1 : 1) : 20;
		TimePoint_t wakeup = sclock::now() + std::chrono::milliseconds(interval);
		std::unique_lock parsing_lock(state.parsing_m);
		state.parsing_cv.wait_until(parsing_lock, wakeup);
	}
}

// ----------------------------------------------------------------------------
// Device Streaming
// ----------------------------------------------------------------------------

void DevicesStartStreaming(ServerState &state)
{
	for (auto &controller : state.controllers)
	{
		{
			auto timeSync = controller->timeSync.contextualLock();
			timeSync->params = TimeSyncParamsForUSB;
			ResetTimeSync(*timeSync);
		}

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
			|| cam->state.contextualRLock()->commState != COMM_SBC_READY))
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
}

void DevicesStopStreaming(ServerState &state)
{
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

	// Reset camera device state
	for (auto &cam : state.cameras)
		cam->receiving = {};
}

// ----------------------------------------------------------------------------
// Realtime Processing
// ----------------------------------------------------------------------------

void RealtimeProcessingThread(std::stop_token stop_token, ServerState *statePtr)
{
	ServerState &state = *statePtr;
	PipelineState &pipeline = state.pipeline;

	SetCurrentThreadName("Realtime Processing");

	while (!stop_token.stop_requested())
	{
		auto frames = pipeline.record.frames.getView();
		if (pipeline.frameNum+1 < frames.endIndex())
		{ // Process most recent fully-received frame (may skip frames!)
			int next = 1;
			for (; pipeline.frameNum+next < frames.endIndex(); next++)
				if (frames[pipeline.frameNum+next])
					break;
			if (pipeline.frameNum+next == frames.endIndex())
				continue; // Just caught while adding
			std::shared_ptr<FrameRecord> frame = frames[pipeline.frameNum+next];
			if (next > 1)
				LOG(LPipeline, LWarn, "Pipeline skipped %d missing/delayed frames!", next-1);
			// Process realtime unless 2 more frames are already waiting, then skip 2
			if (pipeline.frameNum+next+2 < frames.endIndex())
				frame = frames.back(); // new shared_ptr
			assert(frame);

			auto start = sclock::now();
			ProcessFrame(pipeline, frame); // new shared_ptr
			auto end = sclock::now();

			if (state.lowLatencyIMU)
			{
				// TODO: Integrate IMU samples ontop of latest frame for lower latency (2/3)
				// Then IntegrationsSendFrame with updated tracking results
			}

			IntegrationsSendFrame(state.io, state, frame);

			SignalCameraRefresh(0);
		}
		else
		{ // Can wait long, getting notified on any new frame and IMU sample
			TimePoint_t wakeup = sclock::now() + std::chrono::milliseconds(100);
			std::unique_lock processing_lock(state.processing_m);
			state.processing_cv.wait_until(processing_lock, wakeup);

			if (state.lowLatencyIMU && pipeline.frameNum+1 == frames.endIndex())
			{
				// TODO: Integrate IMU samples ontop of latest frame for lower latency (3/3)
				// Then PushTrackingIO without full frame

				SignalCameraRefresh(0);
			}
		}
	}
}

void ProcessStreamFrame(SyncGroup &sync, SyncedFrame &frame, bool premature)
{
	// Frame arrived, possibly incomplete or out-of-date, consider for realtime processing

	if (!premature)
	{ // Implies this if the final time this frame is processed
		// Also the only time it's processed if all cameras sent streaming packets on time


		if (frame.outdated)
		{ // Implies future frames were already complete and fully processed
			// Should always be true if this frame is not complete and only finally processed after a long time
			LOG(LStreaming, LDebug, "--------- Frame ID %d (%d) final processing is outdated!\n", frame.ID, frame.ID&0xFF);
		}

		if (frame.previouslyProcessed && frame.completed == frame.expecting)
		{
			LOG(LStreaming, LDebug, "Finally completing processing of frame ID %d (%d) after %.2fms!\n",
				frame.ID, frame.ID&0xFF, dtMS(frame.SOF, sclock::now()));
		}
		else if (frame.completed != frame.expecting)
		{
			LOG(LStreaming, LDebug, "Finally discarding frame ID %d (%d) after %.2fms!\n",
				frame.ID, frame.ID&0xFF, dtMS(frame.SOF, sclock::now()));
		}
	}

	if (frame.previouslyProcessed)
	{ // Realtime processing currently cannot handle the same frame twice
		// Optimally, we could retrace and improve tracking with new data
		// Especially if tracking was lost

		// TODO: Record into frameRecords anyway for recording
		return;
	}
	// Follow up with realtime processing
	if (frame.outdated)
	{ // Should not happen because it should have gotten a premature processing
		// Except at the very beginning where premature processing conditions have not yet initialised
		if (frame.ID > 50)
			LOG(LStreaming, LDarn, "--------- Frame ID %d (%d) processing is outdated!\n", frame.ID, frame.ID&0xFF);
		// TODO: If latency stdDev is ridiculously large, this might be true even later
		// for frames that had missing packets and were only prematurely processed very late
		return;
	}

	if (premature)
	{ // Implies incomplete, WILL be called another time, either once complete or when discarded
		LOG(LStreaming, LDarn, "Frame %d (%d) has %d/%d cameras completed when prematurely processed!\n", frame.ID, frame.ID&0xFF, frame.completed, frame.expecting);
		for (auto &camera : sync.cameras)
		{
			if (!camera) continue; // Removed while streaming
			if (frame.cameras.size() <= camera->syncIndex) continue; // Newly added after frame
			// - Non-exclusive, might have been a vacant spot, but not an issue
			auto &camRecv = frame.cameras[camera->syncIndex];
			if (!camRecv.announced) continue;
			if (camRecv.erroneous || !camRecv.complete)
			{ // If packet was fully received
				LOG(LStreaming, LTrace, "    Camera %u packet is %s, status %s\n",
					camera->id, camRecv.complete? "complete" : "incomplete", camRecv.erroneous? "error" : "good");
			}
			else
			{
				LOG(LStreaming, LTrace, "    Camera %u has not received any packet despite being announced!\n", camera->id);
			}
		}
	}

	PipelineState &pipeline = GetState().pipeline;

	// Accept for realtime processing, create FrameState

	std::shared_ptr<FrameRecord> frameRecord = std::make_shared<FrameRecord>();
	frameRecord->cameras.resize(pipeline.cameras.size());
	for (auto &camera : sync.cameras)
	{ // Setup camera data
		if (!camera) continue; // Removed while streaming - have to ignore even if data was valid
		if (frame.cameras.size() <= camera->syncIndex) continue; // Newly added after frame
		// - Non-exclusive, might have been a vacant spot, but not an issue
		auto &camRecv = frame.cameras[camera->syncIndex];
		if (!camRecv.announced)
		{
			LOG(LStreaming, LDebug, "--- Camera %u has no blob data announced for frame %d!\n", camera->id, frame.ID);
			continue;
		}
		if (!camRecv.complete)
		{
			LOG(LStreaming, LDarn, "--- Camera %u has incomplete blob data for frame %d!\n", camera->id, frame.ID);
			continue;
		}
		if (camRecv.erroneous)
		{
			LOG(LStreaming, LDarn, "--- Camera %u has erroneous blob data for frame %d!\n", camera->id, frame.ID);
			continue;
		}
		// Packet was properly received
		frameRecord->cameras[camera->pipeline->index] = std::move(camRecv.record);
	}
	for (auto &camera : sync.cameras)
	{ // Copy over image data received before processing started
		if (!camera) continue; // Removed while streaming - have to ignore even if data was valid
		if (camera->receiving.latestFrameImageRecord && camera->receiving.latestFrameImageRecord->frameID == frame.ID)
		{ // May happen if frame processing was unreasonably delayed
			frameRecord->cameras[camera->pipeline->index].image = camera->receiving.latestFrameImageRecord;
		}
	}
	frameRecord->time = frame.SOF;
	frameRecord->timeUTC = convertClock<std::chrono::system_clock::time_point>(frameRecord->time);
	frameRecord->ID = frame.ID;
	// Insert frame into record in the correct order
	auto frames = pipeline.record.frames.getView();
	if (frames.empty() || frames.back()->time < frame.SOF)
		frameRecord->num = pipeline.record.frames.push_back(frameRecord); // new shared_ptr
	else
	{
		// TODO: Ensure frames are processed/appended to frame record in chronological order even between sync groups
		// This is also complicated IF we allow frame IDs to be arbitrary or duplicate between sync groups
		// See note on future frameID plans in record.hpp - space for frames should be pre-determined
		LOG(LStreaming, LError, "Dropped frame %d because frame %d from %.2fms later was already recorded - cannot insert in order!",
			frame.ID, frames.back()->ID, dtMS(frame.SOF, frames.back()->time));
	}

	GetState().processing_cv.notify_one();
}

// ----------------------------------------------------------------------------
// Sync Group Setup
// ----------------------------------------------------------------------------

void SetupSyncGroups(ServerState &state)
{
	auto stream_lock = state.stream.contextualLock();
	// TODO: Setup sync groups in EnsureCamera (based on prior config, e.g. in UI) 2/4
	// Adapt this to use more explicit config and set it up in EnsureCamera & during controller setup

	// Setup sync groups
	std::shared_ptr<Synchronised<SyncGroup>> masterSync;
	for (auto &controller : state.controllers)
	{
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
	}

	// Reset sync group states
	ResetStreamState(*stream_lock);

	if (state.controllers.empty())
	{ // In case no controller is connected, provide virtual sync group for e.g. IMUs
		SetupVirtualSyncGroup(state);
	}

	// Enter cameras into sync group
	for (auto &camera : state.cameras)
	{
		auto &config = state.cameraConfig.getCameraConfig(camera->id);
		if (config.synchronised && camera->controller && camera->controller->sync)
			SetCameraSync(*stream_lock, camera, camera->controller->sync);
		else
			SetCameraSyncNone(*stream_lock, camera, 1000.0f / config.framerate);
	}
}

void SetupVirtualSyncGroup(ServerState &state)
{
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

void DeleteVirtualSyncGroup(ServerState &state)
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
