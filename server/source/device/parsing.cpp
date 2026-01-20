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

#include "parsing.hpp"

#include "tracking_camera.hpp"
#include "tracking_controller.hpp"

#include "server.hpp"
#include "ui/shared.hpp" // Signals
#include "camera_firmware.hpp"

#include "comm/usb.hpp"

#include "util/eigenutil.hpp"
#include "util/log.hpp"
#include "util/image.hpp"

#include "blob/qpu_blob_tiled.hpp"

#include "ctpl/ctpl.hpp"
extern ctpl::thread_pool threadPool;

#include <bitset>


bool ReadSOFPacket(TrackingControllerState &controller, uint8_t *data, int length, TimePoint_t receiveTime)
{
	if (length != SOF_PACKET_SIZE)
	{
		LOG(LSOF, LError, "Received invalid SOF of size %d (!= %d)!\n", length, SOF_PACKET_SIZE);
		return false;
	}
	if (!controller.sync)
	{
		LOG(LSOF, LError, "Controller received SOF but wasn't setup for streaming!\n");
		return false;
	}
	SOFPacket sof = parseSOFPacket(data);
	auto sync_lock = controller.sync->contextualLock();

	// Verify sof.frameID
	FrameID lastSOFID = sync_lock->frames.empty()? 0 : sync_lock->frames.back().ID;
	if (sync_lock->frames.size() > 1)
		lastSOFID = std::max(lastSOFID, std::prev(sync_lock->frames.end(), 2)->ID);
	//sof.frameID = lastSOFID + shortDiff<FrameID, long long>(lastSOFID, sof.frameID, (1<<32)/10, 1<<32);
	// 32Bit doesn't overflow in ~1yr at 144Hz, so not needed until we lower SOF ID bit depth
	if (sof.frameID < lastSOFID)
	{
		LOG(LSOF, LError, "ERROR: Got weird SOF packet with frameID %d < lastSOFID %d\n", sof.frameID, lastSOFID);
	}
	if (lastSOFID > 0 && sof.frameID != lastSOFID+1 && sof.frameID != lastSOFID)
	{ // TODO: Sometimes called every couple dozen frames with the same lastSOFID, verify it is fixed
		// Was probably a frame stuck in controller::sync::frames. not observed in a while, but not sure if its properly fixed
		LOG(LSOF, LWarn, "Skipped %d SOFs from lastSOFID %d to frameID %d!\n", sof.frameID-lastSOFID-1, lastSOFID, sof.frameID);
		for (auto &frame : sync_lock->frames)
		{
			LOG(LSOF, LDebug, "Had SOF %d %fms ago!\n", frame.ID, dtMS(frame.SOF, receiveTime));	
		}
	}

	// Get estimated real-time of SOF with time sync
	TimeSync timeSync = *controller.timeSync.contextualRLock();
	//TimePoint_t timeSOF = GetTimeSynced(timeSync, sof.timeUS, 1<<24, receiveTime);
	uint64_t sofTimestampUS = RebaseSourceTimestamp(timeSync, sof.timeUS, 1<<24, receiveTime);
	TimePoint_t timeSOF = GetTimeSynced(timeSync, sofTimestampUS);
	long dT = dtUS(timeSOF, sclock::now());
	if (dT < -10)
	{ // Sync supposedly happened in the future, time sync has gone bad
		LOG(LSOF, LWarn, "Got bad time sync, SOF is %.2fms in the future, last timestamp %.2fms in the past\n",
			-dT/1000.0f, dtMS(timeSync.lastTime, sclock::now()));
		// NOTE: This previously happened when USB handlers took longer and longer due to logging slowing it down
		// If this happens again, check the USB handler times to make sure they don't increase to unsustainable levels where timesync becomes invalid
		// This is also partially adressed by putting USB packet parsing on a separate thread from the USB callbacks doing the timing
	}

	LOG(LSOF, LDebug, "SOF packet %d with timestamp %dus was %.2fms ago, with time sync delay "
		"avg %dus +- %dus (last timestamp %ldus, %.2fms ago)",
		sof.frameID, sof.timeUS,
		dtMS(timeSOF, sclock::now()),
		(int)(timeSync.diff.avg*1000), (int)(timeSync.diff.stdDev()*1000),
		timeSync.lastTimestamp,
		dtMS(timeSync.lastTime, sclock::now()));
	LOG(LSOF, LDebug, "Changed SOF timestamp from %u (ref %lu) to %lu (ref %lu), "
		"so SOF should be %dus after last timestamp, ""with drift %dus, timesync says %ldus (drift is %.2f%%)\n",
		sof.timeUS, timeSync.lastTimestamp&0xFFFFFF, sofTimestampUS, timeSync.lastTimestamp,
		(int)((long long)sofTimestampUS-timeSync.lastTimestamp),
		(int)((int)((long long)sofTimestampUS-timeSync.lastTimestamp)*(1.0+timeSync.drift)), 
		dtUS(timeSync.lastTime, timeSOF), timeSync.drift*100);

	// Set estimate as SOF for frameID
	RegisterSOF(*sync_lock, sof.frameID, timeSOF);
	return true;
}

bool ReadDebugPacket(TrackingControllerState &controller, uint8_t *data, int length)
{
	if (length == 0) return false;
	//LOG(LControllerDevice, LInfo, "DEBUG of size %d:\n", length);
	int curPos = 0;
	LogLevel curLevel = LDebug;
	for (int i = 0; i < length; i++)
	{
		/* { // Used to verify continuity of debug - needs to be uncommented on controller software
			static const int digitCnt = 2;
			static uint8_t digitPos = 0, digits[digitCnt];
			static int lastNum = 0;

			if (digitPos > 0)
				digits[digitPos++ - 1] = data[i]-'0';
			if (digitPos > digitCnt)
			{
				int num = 0;
				for (int i = 0; i < digitCnt; i++)
					num += digits[i] * (int)std::pow(10, digitCnt-i-1);
				int expNum = ((lastNum+1)%(int)std::pow(10,digitCnt));
				if (expNum != num)
					LOG(LUSB, LWarn, "Num %d is not the expected %d!", num, expNum);
				lastNum = num;
				digitPos = 0;
			}
			if (data[i] == '&')
				digitPos = 1;
		} */
		if (data[i] == '\n' && i != curPos)
		{
			LOG(LControllerDevice, curLevel, "%.*s", i-curPos, (char *)(data+curPos));
			curLevel = LDebug;
			curPos = i+1;
		}
		else if (data[i] < 32)
			data[i] = '*';
		else if (data[i] == '!')
			curLevel = LDarn;
		else if (data[i] == '#')
			curLevel = LError;

	}
	LOG(LControllerDevice, curLevel, "%.*s", length-curPos, (char *)(data+curPos));
	//std::stringstream debugHex;
	//printBuffer(debugHex, data, length);
	//LOG(LControllerDevice, LTrace, "%s", debugHex.str().c_str());
	//LOG(LControllerDevice, LInfo, "DEBUG (%d): %.*s\n", length, length, (char *)data);
	return true;
}

bool ReadEventPacket(TrackingControllerState &controller, uint8_t *data, int length)
{
	if (length == 0) return false;
	int cnt = length/4;
	if (cnt*4 != length)
	{
		LOG(LControllerDevice, LWarn, "Received event packet with size %d (%d bytes remaining)", length, length-cnt*4);
		return false;
	}

	controller.eventLog.delete_culled();

	ControllerEventLog lastEvent = { 0 };
	{
		auto eventLogs = controller.eventLog.getView();
		if (!eventLogs.empty()) lastEvent = eventLogs.back();
	}
	

	uint32_t *events = (uint32_t*)data;
	TimeSync timeSync = *controller.timeSync.contextualRLock(); // Copy to not keep timeSync locked for long
	bool error = false;
	for (int i = 0; i < cnt; i++)
	{
		uint32_t timestamp = events[i] >> 8;
		uint8_t eventCode = events[i] & 0xFF;

		/* if (timestamp == 0)
		{ // Used to verify continuity of events - needs to be uncommented on controller software
			static int lastNum = 0;
			int expNum = ((lastNum+1)%256);
			if (expNum != eventCode)
				LOG(LUSB, LWarn, "Num %d is not the expected %d!", eventCode, expNum);
			lastNum = eventCode;
			continue;
		} */

		ControllerEventLog event;
		// TODO: Change if timestamp becomes sub-us
		event.timestamp = RebaseSourceTimestamp(timeSync, timestamp, 1<<24, sclock::now());
		event.timestampUS = event.timestamp;
		event.syncedTime = GetTimeSynced(timeSync, event.timestamp);

		if (lastEvent.timestampUS > 0)
		{
			long usPassed = dtUS(lastEvent.syncedTime, event.syncedTime);
			if (usPassed < 0)
			{
				LOG(LControllerDevice, LWarn, "Events not continuous! New event is %ldus before last. Might be due to updated time sync", usPassed);
			}
			if (lastEvent.timestampUS > event.timestampUS)
			{
				LOG(LParsing, LError, "Event log timestamps overflowed! First has %ld, new one %ld", lastEvent.timestamp, event.timestamp);
				controller.eventLog.cull_all();
			}
		}

		long dT = dtUS(event.syncedTime, sclock::now());
		if (dT < -10)
		{ // Event supposedly happened in the future, time sync has gone bad
			LOG(LTimesync, LWarn, "Got bad time sync, Event is %.2fms in the future, last timestamp %.2fms in the past"
				" - changed timestamp from %u (ref %lu) to %lu (ref %lu)\n",
				-dT/1000.0f, dtMS(timeSync.lastTime, sclock::now()),
				timestamp, timeSync.lastTimestamp&0xFFFFFF, event.timestamp, timeSync.lastTimestamp);
		}
		else if (dT > 10000)
		{ // Event supposedly happened far in the past
			LOG(LTimesync, LWarn, "Got bad event time, Event is %.2fms in the past, last timestamp %.2fms in the past - maybe host lagged?\n",
				dT/1000.0f, dtMS(timeSync.lastTime, sclock::now()));
			if (dT > 50000)
			{
				break;
				error = true;
			}
		}

		event.id = (ControllerEventID)(eventCode >> 1);
		if (event.id >= CONTROLLER_EVENT_MAX)
		{
			LOG(LControllerDevice, LWarn, "Received invalid event %d with timestamp %lu (%.2fus) - synced %fms ago!", 
				event.id, event.timestamp, event.timestampUS, dtMS(event.syncedTime, sclock::now()));
			error = true;
			continue;
		}
		event.isNewEvent = eventCode & 0x1;

		// That's too much events even for this light logging system
		//LOG(LControllerDevice, LTrace, "Event %d (code %d) has timestamp %lu (%.2fus) - synced %fms ago!", 
		//	event.id, eventCode, event.timestamp, event.timestampUS,
		//	timeSync.measurements < 50? 0.0f : dtMS(event.syncedTime, sclock::now()));
		controller.eventLog.push_back(event);
		lastEvent = event;
	}

	// Blocks of 4096 events each - keep only last 10
	controller.eventLog.cull_front(-100);

	if (lastEvent.timestampUS > 0)
	{ // Leave event::XXXus as marking/link to jump to that timestamp
		LOG(LControllerDevice, LTrace, "Received %d events%s, last parsed at event::%ldus\n", cnt, error? " with errors" : "", lastEvent.timestamp);
	}

	SignalServerEvent(EVT_UPDATE_EVENTS);
	return !error;
}

bool ReadStatusPacket(ServerState &state, TrackingControllerState &controller, uint8_t *data, int length)
{
	const int CONTROLLER_STATUS_SIZE = 8;
	const int PORT_STATUS_SIZE = 7;
	if (length < CONTROLLER_STATUS_SIZE)
	{
		LOG(LControllerDevice, LError, "Received malformed status packet of length %d < %d!\n", length, CONTROLLER_STATUS_SIZE);
		return false;
	}
	std::shared_lock dev_lock(state.deviceAccessMutex);

	controller.status.flags = (ControllerStatusFlags)data[0];
	controller.status.syncCfg = (ControllerSyncConfig)data[1];
	uint8_t reserved = data[2];
	uint8_t portCount = data[3] >> 4;
	controller.status.powerState = (ControllerPowerState)(data[3] & 0xF);
	controller.status.voltagePD.update((data[4] << 8 | data[5]) / 1000.0f);
	controller.status.voltageExt.update((data[6] << 8 | data[7]) / 1000.0f);

	if (length != (CONTROLLER_STATUS_SIZE + portCount*PORT_STATUS_SIZE))
	{
		LOG(LControllerDevice, LError, "Received malformed data of length %d, port count %d!\n",
			length, portCount);
		return false;
	}
	if (state.isStreaming != (bool)(controller.status.flags & CONTROLLER_COMM_CHANNELS) ||
		state.isStreaming != (bool)(controller.status.flags & CONTROLLER_TIME_SYNC) ||
		(!state.isStreaming && controller.status.syncCfg != SYNC_CFG_NONE))
	{ // TODO: Spammed when state changes, so make sure the last change is actually some time ago
		// E.g. if this is right after streaming started, but controller hasn't been fully instructed yet
		// However there is a valid use case, e.g. when host dropped out (lagged, went to sleep, etc.)
		// The controller has a timeout so it might silently stop streaming in these cases, which to the host is unexpected once it wakes up
		LOG(LControllerDevice, LDarn, "Potentially erroneous state! %sstreaming, state: comm channels %s, time sync %s, controller %ssynced!\n",
			state.isStreaming? "" : "not ", (controller.status.flags & CONTROLLER_COMM_CHANNELS)? "open" : "closed", (controller.status.flags & CONTROLLER_TIME_SYNC)? "on" : "off",
			(controller.status.syncCfg != SYNC_CFG_NONE)? "" : "not ");
	}
	// TODO: Double check sync configuration the same way?

	// Step 1: Check addition or removal of cameras
	int camsConnected = 0, camsConnecting = 0, newCamsConnecting = 0;
	std::vector<std::pair<int32_t,int>> addedCams, removedCams;
	for (int i = 0; i < portCount; i++)
	{
		bool existing = controller.cameras.size() > i && controller.cameras[i] != NULL;
		uint8_t *portState = data + CONTROLLER_STATUS_SIZE + i*PORT_STATUS_SIZE;
		ControllerCommState portComms = (ControllerCommState)portState[0];
		static_assert(sizeof(CameraID) == 4);
		CameraID id = *(CameraID*)(portState+3);
		if ((portComms&COMM_READY) == COMM_READY)
		{
			if (portComms == COMM_SBC_READY)
				camsConnected++;
			else if (portComms == COMM_MCU_READY)
				camsConnecting++;
			if (id == 0 && !existing) newCamsConnecting++;
			if (id == 0) continue;
			// Cannot add camera without knowing its ID
			if (!existing)
			{ // Setup new tracking camera
				addedCams.emplace_back(id, i);
			}
			else if (id != 0 && controller.cameras[i]->id != id)
			{ // Shouldn't happen, but let's be safe
				addedCams.emplace_back(id, i);
				removedCams.emplace_back(controller.cameras[i]->id, i);
			}
		}
		else if (existing)
		{
			// TODO: CameraCheckDisconnected amd ReadStatusPacket both check timeouts before camera is removed (1/2)
			// This here retains the controller association, might not be necessary
			auto camState = *controller.cameras[i]->state.contextualRLock();
			if (dtMS(camState.lastConnecting, sclock::now()) < 1000)
				continue; // Show camera reconnecting message (might be switching between MCU and Pi)
			if (dtMS(camState.lastConnected, sclock::now()) < 5000)
				continue; // Show camera lost message
			if (camState.error.encountered &&
				dtMS(camState.error.time, sclock::now()) < 10000)
				continue; // Show camera error message
			// Might want to further defer removal of errored camera to give it a chance to recover / restart
			// Detach from controller
			removedCams.emplace_back(controller.cameras[i]->id, i);
		}
	}

	// Step 1: Apply addition or removal of cameras under unique_lock
	if (controller.cameras.size() != portCount || !addedCams.empty() || !removedCams.empty())
	{
		std::unique_lock dev_lock(state.deviceAccessMutex); // controller.cameras, controller.packetState,
		if (controller.ports.size() != portCount)
		{ // Adopt controller port count
			if (!controller.ports.empty())
			{
				LOG(LControllerDevice, LError, "Controller status packet reports port count %d when previously %d where reported! "
					"This is not allowed!\n", portCount, (int)controller.ports.size());
				return false;
			}
			controller.cameras.resize(portCount);
			controller.ports.resize(portCount);
		}
		// Detect moved cameras
		int camsLost = 0, camsGained = 0, camsMoved = 0;
		for (auto rem : removedCams)
		{
			auto add = addedCams.begin();
			while (add != addedCams.end() && add->first != rem.first) add++;
			if (add != addedCams.end())
			{ // Camera moved from one port to the next! Magic!
				// Reassign port - if something was replaced, it too is either moved or detached
				controller.cameras[add->second] = controller.cameras[rem.second];
				controller.cameras[add->second]->port = add->second;
				controller.cameras[rem.second] = NULL;
				camsMoved++;
				addedCams.erase(add);
			}
			else
			{ // Detach from controller
				controller.cameras[rem.second]->controller = NULL;
				controller.cameras[rem.second]->port = -1;
				if (CameraCheckDisconnected(state, *controller.cameras[rem.second]))
					camsLost++;
				controller.cameras[rem.second] = NULL;
				LOG(LDefault, LInfo, "Removed Camera with ID %d from port %d!\n", rem.first, rem.second);
			}
		}
		for (auto add : addedCams)
		{ // Genuinely newly connected, not moved
			camsGained++;
			auto contIt = std::find_if(state.controllers.begin(), state.controllers.end(),
				[&](auto &cont){ return cont.get() == &controller; });
			if (contIt == state.controllers.end())
			{
				LOG(LDefault, LError, "Could not find controller when adding camera - was it removed already?\n");
				return false;
			}
			std::shared_ptr<TrackingCameraState> camera = EnsureCamera(state, add.first, *contIt);
			camera->controller = *contIt; // Set both in EnsureCamera and here in case camera existed already
			camera->port = add.second;
			LOG(LDefault, LInfo, "Added Camera with ID %d on port %d!\n", camera->id, camera->port);
			controller.cameras[add.second] = std::move(camera);
		}
		LOG(LControllerDevice, LDebug, "Controller: %d/%d cameras connected, %d connecting, %d new. Total: Lost %d and gained %d cameras, %d connected!\n",
			camsConnected, portCount, camsConnecting, newCamsConnecting, camsLost, camsGained, (int)state.cameras.size());
		SignalServerEvent(EVT_UPDATE_CAMERAS);
	}
	else if (controller.newCamerasConnecting != newCamsConnecting)
	{
		LOG(LControllerDevice, LDebug, "Controller: %d/%d cameras connected, %d connecting, %d new. Total: %d!\n",
			camsConnected, portCount, camsConnecting, newCamsConnecting, (int)state.cameras.size());
		SignalServerEvent(EVT_UPDATE_CAMERAS);
	}
	else
	{
		LOG(LControllerDevice, LTrace, "Controller: %d/%d cameras connected, %d connecting, %d new. Total: %d!\n",
			camsConnected, portCount, camsConnecting, newCamsConnecting, (int)state.cameras.size());
	}
	// TODO: Rework connecting cameras (1/2)
	controller.newCamerasConnecting = newCamsConnecting;

	// Step 3: Update camera states
	bool updatedCameras = false;
	for (int i = 0; i < portCount; i++)
	{
		if (!controller.cameras[i]) continue;
		auto camState = controller.cameras[i]->state.contextualLock();
		// TODO: Change camera iteration packet to include version of cameras? Or implement a separate query?
		uint8_t *portState = data + CONTROLLER_STATUS_SIZE + i*PORT_STATUS_SIZE;
		ControllerCommState portComms = (ControllerCommState)portState[0];
		ControllerPortStatus portStatus = (ControllerPortStatus)portState[1];
		uint8_t reserved = portState[2];
		if (portComms == COMM_SBC_READY)
		{ // Clear any possible error state
			if (camState->error.encountered && dtMS(camState->error.time, sclock::now()) > 1000)
			{ // Might need this overlap protection to prevent race-conditions
				camState->error.recoverTime = sclock::now();
				camState->error.recovered = true;
				camState->error.encountered = false;
				updatedCameras = true;
			}
		}
		if (portComms == COMM_SBC_READY) camState->hadPiConnected = true;
		else if (portComms == COMM_MCU_READY) camState->hadMCUConnected = true;
		if ((portComms&COMM_READY) == COMM_READY) camState->lastConnected = sclock::now();
		else if (portComms != COMM_NO_CONN) camState->lastConnecting = sclock::now();
		if (camState->commState != portComms)
			updatedCameras = true;
		camState->commState = portComms;
	}
	if (updatedCameras)
		SignalServerEvent(EVT_UPDATE_INTERFACE);
	return true;
}

CameraFrameRecord ReadStreamingPacket(TrackingCameraState &camera, const PacketHeader header, const uint8_t *data, int length, bool erroneous)
{
	CameraFrameRecord frame = {};
	if (erroneous)
		return frame;

	// Determine packet structure
	const int blobEncSize = 6;
	int totalBlobCount = (length-STREAM_PACKET_HEADER_SIZE) / STREAM_PACKET_BLOB_SIZE;
	if (totalBlobCount*STREAM_PACKET_BLOB_SIZE + STREAM_PACKET_HEADER_SIZE != length)
	{
		LOG(LParsing, LWarn, "Invalid blob packet size of %d not divisible by %d!",
			(int)length - STREAM_PACKET_HEADER_SIZE, STREAM_PACKET_BLOB_SIZE);
		return frame;
	}

	// Parse array of blobs
	frame.rawPoints2D.reserve(totalBlobCount);
	frame.properties.reserve(totalBlobCount);
	const uint8_t *blobData = data+STREAM_PACKET_HEADER_SIZE;
	for (int b = 0; b < totalBlobCount; b++)
	{ // Parse all data as blobs
		int base = b*STREAM_PACKET_BLOB_SIZE;
		Eigen::Vector2f pt(
			(float)*(uint16_t*)&blobData[base+0] / std::numeric_limits<uint16_t>::max(), 
			(float)*(uint16_t*)&blobData[base+2] / std::numeric_limits<uint16_t>::max());
		pt = npix2cam(camera.pipeline->mode, pt); // Convert from normalised pixel space (0-1) to our coordinate system
		frame.rawPoints2D.push_back(pt);
		BlobProperty prop;
		prop.size = (float)*(uint8_t*)&blobData[base+4] / camera.pipeline->mode.widthPx;
		prop.value = *(uint8_t*)&blobData[base+5] * 4;
		frame.properties.push_back(prop);
		static_assert(STREAM_PACKET_BLOB_SIZE == 6);
	}
	frame.received = true;
	return frame;
}

bool ReadErrorPacket(TrackingCameraState &camera, const PacketHeader header, const uint8_t *data, int length, bool erroneous)
{
	TrackingCameraState::Errors error = {};
	error.encountered = true;
	error.recovered = false;
	error.time = sclock::now();
	if (erroneous || length < 2)
	{
		LOG(LCameraDevice, LError, "Received invalid error packet of length %d from camera %d in port %d!\n",
			(int)length, camera.id, camera.port);
		error.code = ERROR_UNKNOWN;
		error.serious = true;
	}
	else
	{
		error.code = data[0] >= ERROR_MAX? ERROR_UNKNOWN : (ErrorTag)data[0];
		error.serious = (bool)data[1];
	}

	std::vector<std::string> backtrace;
	if (length > 2)
	{
		std::size_t pos = 2;
		while (pos < length)
		{ // Copy line-by-line, could also directly print
			char *start = (char*)&data[pos];
			std::size_t len = strnlen(start, length-pos);
			backtrace.push_back(std::string(start, start+len));
			pos += len+1; // Include null-termination
		}
	}

	// Implicitly assume mode change
	if (error.serious)
		camera.recvModeSet(TRCAM_STANDBY);

	// Update error state
	camera.state.contextualLock()->error = error;

	if (error.code == ERROR_UNKNOWN)
	{
		LOG(LCameraDevice, LError, "Received invalid error from camera %d in port %d!\n",
			camera.id, camera.port);
	}
	else
	{
		LOG(LCameraDevice, LError, "Received %s error from camera %d in port %d: %s (%d)\n",
			error.serious? "serious" : "regular", camera.id, camera.port, ErrorTag_String[error.code], (int)error.code);
	}
	for (auto &func : backtrace)
	{
		LOG(LCameraDevice, LError, "%s", func.c_str());
	}

	// We will keep camera, but it will be in error.encountered state until it restarts comms
	// Then it will be in error.recovered state until we tell it to start streaming again

	if (camera.controller && camera.controller->comm)
		comm_submit_control_request(camera.controller->comm, COMMAND_IN_STATUS, 0, 0);
	return true;
}

bool ReadModePacket(TrackingCameraState &camera, const PacketHeader header, const uint8_t *data, int length, bool erroneous)
{
	if (erroneous) return false;
	if (length < 1) return false;
	camera.recvModeSet(data[0]);
	return true;
}

bool ReadVisualPacket(TrackingCameraState &camera, const PacketHeader header, const uint8_t *data, int length, bool erroneous)
{
	if (erroneous) return false;
	const int metaSize = 2*4 + 2*2 + 2 + 2; // Bounds, Center, Size, Refinement Method
	if (length <= metaSize) return false;

	LOG(LParsing, LDebug, "Parsing visualdebug packet of size %d", (int)length);

	// Parse metadata
	uint16_t *metaData = (uint16_t*)data;
	Bounds2i bounds(metaData[0], metaData[1], metaData[2], metaData[3]);
	Eigen::Vector2f centerPos = Eigen::Vector2f(metaData[4] / 65536.0f, metaData[5] / 65536.0f);
	centerPos = npix2cam(camera.pipeline->mode, centerPos); // Convert from normalised pixel space (0-1) to our coordinate system
	float size = metaData[6] / 65536.0f * 256.0f;
	int refinementMethod = metaData[7] >> 8;
	int dbgPtCount = metaData[7] & 0xFF;

	// Verify, especially bounds
	const int MAX_IMAGE_SIZE = 4096;
	Eigen::Vector2i extends = bounds.extends();
	if (extends.x() <= 0 || extends.y() <= 0 || (extends.x() * extends.y() > MAX_IMAGE_SIZE))
	{
		LOG(LParsing, LError, "Bounds are invalid! (%dx%d)\n", extends.x(), extends.y());
		return false;
	}
	int imgSize = extends.x() * extends.y();
	int bitSize = imgSize / 8;
	if (imgSize & 7) bitSize++;
	int ptSize = dbgPtCount * 3;
	if (length != (metaSize + imgSize + bitSize + ptSize))
	{
		LOG(LParsing, LError, "VisualDebug packet size of %d doesn't match derived size of %d! Split as %d, %d, %d, %d\n",
			(int)length, metaSize + imgSize + bitSize + ptSize,
			metaSize, imgSize, bitSize, ptSize);
		return false;
	}
	if (dbgPtCount > 0 && !(refinementMethod == 2 || refinementMethod == 3))
	{
		LOG(LParsing, LError, "VisualDebug packet includes %d debug points, but declared refinement method %d should not include any!\n",
			dbgPtCount, refinementMethod);
		return false;
	}
	// Debug
	LOG(LParsing, LDebug, "Parsed meta. %dx%d area, bounds (%d, %d, %d, %d), %f size, center pos (%f, %f)/(%fpx, %fpx)\n",
		extends.x(), extends.y(), bounds.min.x(), bounds.min.y(), bounds.max.x(), bounds.max.y(), size,
		centerPos.x(), centerPos.y(), (centerPos.x()+1)*PixelFactor, (centerPos.y()+camera.pipeline->mode.aspect)*PixelFactor);

	// Read image, flipped on Y-axis
	std::vector<uint8_t> image(imgSize);
	const uint8_t *imgData = data + metaSize;
	std::vector<uint8_t> flippedImg(imgSize);
	for (int y = 0; y < extends.y(); y++)
		memcpy(image.data()+(extends.y()-y-1)*extends.x(), image.data()+y*extends.x(), extends.x());

	// Read bitmask
	std::bitset<MAX_IMAGE_SIZE> bitmask;
	const uint8_t *bitData = data + metaSize + imgSize;
	for (int b = 0; b < bitSize; b++)
	{ // Parse bit mask byte by byte (if imgSize%8 != 0, remaining bits don't matter anyway)
		for (int i = 0; i < 8; i++)
			bitmask.set(b*8 + i, (bitData[b] >> i) & 1);
	}
	if (!bitmask.any())
	{ // Should not happen
		LOG(LParsing, LWarn, "VisualDebug with no blob mask!\n");
	}
	// Copy to mask grayscale
	std::vector<uint8_t> mask(extends.prod());
	for (int y = 0; y < extends.y(); y++)
		for (int x = 0; x < extends.x(); x++)
			mask[y*extends.x()+x] = bitmask.test((extends.y()-y-1)*extends.x()+x)? 255 : 0;

	// Parse debug points
	std::vector<Eigen::Vector2f> boundPoints(dbgPtCount);
	const uint8_t *pointData = data + metaSize + imgSize + bitSize;
	for (int p = 0; p < dbgPtCount; p++)
	{
		uint32_t ptData = (pointData[p*3+0] << 16) | (pointData[p*3+1] << 8) | pointData[p*3+2];
		uint8_t outlier = ptData & 0b11;
		uint16_t y = (ptData >> 2) & 0x7FF;
		uint16_t x = (ptData >> 13) & 0x7FF;
		boundPoints.push_back(Eigen::Vector2f(x / 2047.0f * extends.x() + 0.5f, - (y / 2047.0f * extends.y() + 0.5f)));
	}

	{ // Move parsed data to camera state
		auto visual_lock = camera.receiving.visualDebug.contextualLock();
		if (visual_lock->hasBlob && visual_lock->bounds.overlaps(bounds))
		{ // Overlap, move anchor point with centers (not real centers, both offset by half a pixel)
			visual_lock->offset += bounds.center<float>() - visual_lock->bounds.center<float>();
			// Viz will reset when it hits the borders
		}
		else
		{
			visual_lock->offset = Eigen::Vector2f::Zero();
		}
		visual_lock->lastBounds = visual_lock->bounds;
		visual_lock->mask = std::move(mask);
		visual_lock->image = std::move(image);
		visual_lock->pos = centerPos;
		visual_lock->size = size;
		visual_lock->refinementMethod = refinementMethod;
		visual_lock->dbgPtCount = dbgPtCount;
		visual_lock->boundPoints = boundPoints;
		visual_lock->bounds = bounds;
		visual_lock->hasBlob = true;
	}
	SignalCameraRefresh(camera.id);
	return true;
}

bool ReadFramePacket(TrackingCameraState &camera, const PacketHeader header, const uint8_t *data, int length, bool erroneous)
{
	if (!camera.config.imageStreaming.enabled)
	{ // TODO: This is bad if it happens, so should log, but make sure to only log it after repeated violation
		LOG(LCameraDevice, LInfo, "Camera %d is still sending frames after disabling frame streaming!", camera.id);
		return false;
	}
	const int headerSize = 22;
	if (length < headerSize)
	{ // Erroneous packet, but can't be sure it is for the current frame being received
		//camera.state.frameNext.contextualLock()->get()->erroneous = true;
		return false;
	}

	// Parse header data
	uint8_t *headerData = (uint8_t*)data;
	uint32_t imageSize = *(uint32_t*)&headerData[0];
	uint32_t blockOffset = *(uint32_t*)&headerData[4];
	uint32_t blockSize = *(uint32_t*)&headerData[8];
	uint8_t quality = *(uint8_t*)&headerData[12];
	uint8_t subsample = *(uint8_t*)&headerData[13];
	Bounds2i imageBounds(
		*(uint16_t*)&headerData[14],
		*(uint16_t*)&headerData[16],
		*(uint16_t*)&headerData[18],
		*(uint16_t*)&headerData[20]);
	uint32_t imageWidth = subsample == 0? 0 : imageBounds.extends().x()/subsample;
	uint32_t imageHeight = subsample == 0? 0 : imageBounds.extends().y()/subsample;

	// Check parsing state
	auto &parseImg = camera.receiving.parsingFrameImage;
	if ((TruncFrameID)(parseImg.frameID) != header.frameID)
	{ // Start new frame
		if (parseImg.received < parseImg.jpeg.size())
		{ // Currently only support parsing one frame at a time
			LOG(LParsing, LWarn, "Camera %d had only received %d/%d bytes for image of frame %d!",
				camera.id, parseImg.received, (int)parseImg.jpeg.size(), parseImg.frameID);
		}

		parseImg.frameID = camera.sync? EstimateFullFrameID(*camera.sync->contextualLock(), header.frameID) : header.frameID;
		parseImg.received = 0;
		parseImg.erroneous = erroneous || imageWidth == 0 || imageHeight == 0;
		parseImg.jpeg.clear();
		if (!erroneous) // Allocated data
			parseImg.jpeg.resize(imageSize, 0);
		LOG(LParsing, LDebug, "Camera %d starts receiving new image (%d x %d, %d bytes) of frame %d",
			camera.id, imageWidth, imageHeight, imageSize, parseImg.frameID);
	}

	if (erroneous)
	{ // If a single packet is erronous (e.g. checksum failed), the image cannot be recovered currently
		LOG(LParsing, LDarn, "Camera %d received erroneous packet with block (offset %d length %d) for image (%d x %d, %d bytes) of frame %d!",
			camera.id, blockOffset, blockSize, imageWidth, imageHeight, imageSize, parseImg.frameID);
		parseImg.erroneous = true;
	}

	// Register
	parseImg.received += blockSize;
	if (parseImg.erroneous)
		return false;

	// Error-check block
	if (imageWidth > 1280 || imageHeight > 800 || imageSize != parseImg.jpeg.size())
	{
		LOG(LParsing, LWarn, "Camera %d received invalid image header (%d x %d, %d bytes) for %d bytes image of frame %d!",
			camera.id, imageWidth, imageHeight, imageSize, (int)parseImg.jpeg.size(), parseImg.frameID);
		parseImg.erroneous = true;
		return false;
	}
	if ((headerSize+blockSize) != length || (blockOffset+blockSize) > parseImg.jpeg.size())
	{
		LOG(LParsing, LWarn, "Camera %d received block (offset %d length %d) for image (%d x %d, %d bytes) of frame %d!",
			camera.id, blockOffset, blockSize, imageWidth, imageHeight, imageSize, parseImg.frameID);
		parseImg.erroneous = true;
		return false;
	}
	if (parseImg.jpeg[blockOffset] != 0)
	{
		LOG(LParsing, LWarn, "Camera %d received block (offset %d length %d) overlapping with existing data of image (%d x %d, %d bytes) of frame %d!",
			camera.id, blockOffset, blockSize, imageWidth, imageHeight, imageSize, parseImg.frameID);
		parseImg.erroneous = true;
		return false;
	}

	// Copy block of jpeg data
	memcpy(&parseImg.jpeg[blockOffset], &data[headerSize], blockSize);
	LOG(LParsing, LDebug, "Camera %d received block (offset %d length %d) in image packet of size %d with %d / %d bytes received in total!",
		camera.id, blockOffset, blockSize, (int)length, parseImg.received, (int)parseImg.jpeg.size());

	if (parseImg.received == parseImg.jpeg.size() && !parseImg.erroneous)
	{ // Got full jpeg data for frame image, store record

		// Compile image record with all info necessary to decompress and use the camera image
		auto imageRecord = std::make_shared<CameraImageRecord>();
		imageRecord->cameraID = camera.id;
		imageRecord->frameID = parseImg.frameID;
		imageRecord->boundsPx = imageBounds;
		// Record frame size because camera mode might have changed since frame was captured
		imageRecord->frameX = camera.pipeline->mode.widthPx;
		imageRecord->frameY = camera.pipeline->mode.heightPx;
		imageRecord->imageX = imageWidth;
		imageRecord->imageY = imageHeight;
		imageRecord->jpeg = std::move(parseImg.jpeg);

		// Asnynchronously decompress camera image record
		threadPool.push([&camera](int, std::shared_ptr<CameraImageRecord> &imageRecord)
		{
			auto image = decompressCameraImageRecord(imageRecord);
			if (!image) return; // Image jpeg is faulty, don't store record

			std::shared_lock device_lock(GetState().deviceAccessMutex);
			auto camera = GetCamera(GetState(), imageRecord->cameraID);
			if (!camera) return; // Camera has been disconnected

			if (GetState().pipeline.keepFrameImages)
			{ // Store compressed image record in frameRecord for later use
				auto framesRecord = GetState().pipeline.record.frames.getView<false>();
				if (framesRecord.empty()) return;
				// Search for frame with correct ID in recently recorded frames. Cannot assume ID == num, but can assume image is recent
				auto frameRecord = std::prev(framesRecord.end());
				while (frameRecord != framesRecord.begin() && (!*frameRecord || frameRecord->get()->ID > imageRecord->frameID)) frameRecord--;
				if (!*frameRecord || frameRecord->get()->ID != imageRecord->frameID)
				{ // TODO: Handle image recording when frame has not (yet) been recorded
					if (framesRecord.back() && framesRecord.back()->ID < imageRecord->frameID)
					{ // Can happen if image image was received before frame processing started (e.g. waiting for another camera)
						LOG(LParsing, LDarn, "Failed to record received camera image, frame ID %d has not yet been recorded!", imageRecord->frameID);
					}
					else
					{ // May rarely happen if a frame is dropped / stream packets are invalid so frame processing didn't (yet) happen
						LOG(LParsing, LWarn, "Failed to record received camera image for unknown frame ID %d!", imageRecord->frameID);
					}
				}
				else
				{ // Frame processing already started, can record directly
					auto &cameraRecord = frameRecord->get()->cameras[camera->pipeline->index];
					cameraRecord.image = imageRecord; // new shared_ptr
				}
			}

			// Store as most recent decompressed image
			camera->receiving.latestFrameImage = std::move(image);
			camera->receiving.latestFrameImageRecord = std::move(imageRecord);

			SignalCameraRefresh(camera->id);
		}, std::move(imageRecord));

		return true;
	}

	return true;
}

std::shared_ptr<CameraImage> decompressCameraImageRecord(std::shared_ptr<CameraImageRecord> &imageRecord)
{
	int width, height;
	std::vector<uint8_t> imageBuf;
	if (!decompress(imageBuf, imageRecord->jpeg.data(), imageRecord->jpeg.size(), width, height)
		|| width != imageRecord->imageX || height != imageRecord->imageY)
	{
		LOG(LParsing, LWarn, "Camera %d frame image failed to decompress image of size %dx%d from %dbytes!\n",
			imageRecord->cameraID, imageRecord->imageX, imageRecord->imageY, (int)imageRecord->jpeg.size());
		return nullptr;
	}
	LOG(LParsing, LDebug, "Camera %d received full image of size %d (decompressed %d) for imageRecord ID %d!\n",
		imageRecord->cameraID, (int)imageRecord->jpeg.size(), (int)imageBuf.size(), imageRecord->frameID);

	// No need for synchronisation (hopefully), pointer switch
	auto image = std::make_shared<CameraImage>();
	image->frameID = imageRecord->frameID;
	image->boundsPx = imageRecord->boundsPx;
	// TODO: If mode.widthPx != mode.sensorWidth, it might be wrong depending on what the camera is sending
	Bounds2f fullFrame(0, 0, imageRecord->frameX, imageRecord->frameY);
	image->boundsRel.min = (image->boundsPx.min.cast<float>() - fullFrame.min).cwiseQuotient(fullFrame.extends());
	image->boundsRel.max = (image->boundsPx.max.cast<float>() - fullFrame.min).cwiseQuotient(fullFrame.extends());
	image->image = std::move(imageBuf);
	image->width = imageRecord->imageX;
	image->height = imageRecord->imageY;
	image->jpegSize = imageRecord->jpeg.size();
	return image;
}

bool ReadStatPacket(TrackingCameraState &camera, const PacketHeader header, const uint8_t *data, int length, bool erroneous)
{
	if (erroneous) return false;
	if (length != STAT_PACKET_SIZE) // && writeStatLogs
	{
		LOG(LCameraDevice, LWarn, "StatPacket received was of size %d != %d!\n", (int)length, STAT_PACKET_SIZE);
		return false;
	}
	auto stat_lock = camera.receiving.statistics.contextualLock();
	uint32_t lastStatFrame = stat_lock->header.frame;
	*stat_lock = parseStatPacket(data);
	auto &s = *stat_lock;
	float elapsedS = s.header.deltaUS/1000000.0f;
	LOG(LCameraDevice, LInfo, "Camera %d, Frame %d, %.2f fps, %.1f\u00B0C\n", camera.id, s.header.frame,
		(s.header.frame-lastStatFrame)/elapsedS, s.header.tempSOC/100.0f);
	LOG(LCameraDevice, LInfo, "   Latency %.2fms (qpu %.2f, fetch %.2f, CCL %.2f, "
		"post %.2f, TX %.2f), Vis: (%dus, %.0fHz), Stream: (%dms, %.0fHz)\n",
		s.times.latency/1000.0f, s.times.qpu/1000.0f, s.times.fetch/1000.0f,
		s.times.ccl/1000.0f, s.times.post/1000.0f, s.times.send/1000.0f,
		s.incidents.vis.avg, s.incidents.vis.occurences/elapsedS,
		(s.incidents.stream.avg<<8)/1000, s.incidents.stream.occurences/elapsedS);
	LOG(LCameraDevice, LDebug, "       Of %.2fms total post-processing, tasks took: "
		"blurring %.2fms maxima hints %.2fms (%d+%d+%d), resegmentation %.2fms, refinement %.2fms\n",
		s.times.post/1000.0f, s.times.blur/1000.0f,
		s.times.maxima/1000.0f, s.times.local, s.times.iter, s.times.check,
		s.times.reseg/1000.0f, s.times.refine/1000.0f);
	if (s.incidents.lag.occurences > 0 || s.header.skipQPU > 0 || s.header.skipCPU > 0)
	{ // To interpret: First QPU drops, then CPU drops, of the 1000 remaining # lagged
		LOG(LCameraDevice, LInfo, "   %d QPU-dropped, %d CPU-dropped, %d lagged (avg %.2fms, max %.2fms)\n",
			s.header.skipQPU, s.header.skipCPU, s.incidents.lag.occurences, s.incidents.lag.avg/1000.0f, s.incidents.lag.max/1000.0f);
	}
	{ // Log incidents in detail
		auto printIncidents = [](auto &stat, const char *label)
		{
			if (stat.occurences)
				LOG(LCameraDevice, LDebug, "    %dx %s %dus, max %dus\n", stat.occurences, label, stat.avg, stat.max);
		};
		printIncidents(s.incidents.await, "await");
		printIncidents(s.incidents.skip, "skip");
		printIncidents(s.incidents.access, "access");
		printIncidents(s.incidents.proc, "proc");
		printIncidents(s.incidents.handle, "handle");
		printIncidents(s.incidents.cpuwait, "CPUwait");
	}
	return true;
}

bool ReadWirelessPacket(TrackingCameraState &camera, const PacketHeader header, const uint8_t *data, int length, bool erroneous)
{
	if (erroneous) return false;
	if (length < WIRELESS_PACKET_HEADER) return false;
	auto &wireless = camera.config.wireless;

	bool newlyConnected = wireless.wifiStatus != WIRELESS_STATUS_CONNECTED;
	WirelessConfig prevConfig = wireless.lastConfig;
	wireless.lastConfig = (WirelessConfig)data[0];
	wireless.wifiStatus = (WirelessStatus)(data[1] & WIRELESS_STATUS_MASK);
	wireless.sshStatus = (WirelessStatus)(data[2] & WIRELESS_STATUS_MASK);
	wireless.serverStatus = (WirelessStatus)(data[3] & WIRELESS_STATUS_MASK);
	wireless.wifiFlags = (WirelessStatus)(data[1] & WIRELESS_STATUS_FLAGS);
	wireless.sshFlags = (WirelessStatus)(data[2] & WIRELESS_STATUS_FLAGS);
	wireless.serverFlags = (WirelessStatus)(data[3] & WIRELESS_STATUS_FLAGS);
	// 1 free byte for future use
	uint8_t errorLen = data[5];
	uint8_t SSIDLen = data[6];
	uint8_t IPLen = data[7];

	if (wireless.lastConfig != prevConfig)
	{ // Config changed. Ideally, setConfig should already be lastConfig
		// But sometimes, cameras changes more configs than requested (e.g. when disabling wifi), or changes on its own
		wireless.setConfig = wireless.lastConfig;
	}

	const uint8_t *ptr = data + WIRELESS_PACKET_HEADER;
	if (errorLen > 0)
	{
		wireless.error = std::string((char*)ptr, errorLen);
		wireless.errorTime = sclock::now();
		ptr += errorLen;
	}
	if (SSIDLen > 0)
	{
		wireless.SSID = std::string((char*)ptr, SSIDLen);
		wireless.errorTime = sclock::now();
		ptr += SSIDLen;
	} else if (newlyConnected) wireless.IP.clear();
	if (IPLen > 0)
	{
		wireless.IP = std::string((char*)ptr, IPLen);
		wireless.errorTime = sclock::now();
		ptr += IPLen;
	} else if (newlyConnected) wireless.IP.clear();

	SignalServerEvent(EVT_UPDATE_INTERFACE);
	return true;
}

bool ReadBGTilesPacket(TrackingCameraState &camera, const PacketHeader header, const uint8_t *data, int length, bool erroneous)
{
	if (erroneous) return false;
	if (length < 3) return false;
	// TODO: Replace extends in packet with tilePx + offsetPx
	uint8_t extendsX = data[0], extendsY = data[1];
	const float tilePx = 8;
	ProgramLayout layout = SetupProgramLayout(camera.pipeline->mode.widthPx, camera.pipeline->mode.heightPx, 8, false);
	Eigen::Vector2i offsetPx = layout.maskOffset;
	const uint8_t *tiles = &data[2];
	uint16_t tileCnt = ((uint16_t)length-2)/2;
	LOG(LCameraDevice, LDebug, "Received a total of %d background tiles! Range %d, %d; Offset %d, %d\n",
		tileCnt, extendsX, extendsY, offsetPx.x(), offsetPx.y());
	// Update background tiles
	auto bg_lock = camera.receiving.background.contextualLock();
	//bg_lock->tiles.clear();
	for (int i = 0; i < tileCnt; i++)
	{
		float x = (tiles[i*2+0]*tilePx + offsetPx.x() + tilePx/2) * 2.0f/camera.pipeline->mode.widthPx - camera.pipeline->mode.sizeW;
		float y = (tiles[i*2+1]*tilePx + offsetPx.y() + tilePx/2) * 2.0f/camera.pipeline->mode.widthPx - camera.pipeline->mode.sizeH;
		bg_lock->tiles.emplace_back(x, -y);
	}
	// TODO: Figure out why this point scale factor is necessary, and just for tiles
	//float scaleFactor = (float)layout.maskSize.y()/layout.maskSize.x(); // 800/1152 - Not quite
	float scaleFactor = 1.0f;//0.69f; // Seems exactly right, really confused
	bg_lock->tileSize = tilePx / camera.pipeline->mode.widthPx * scaleFactor;
	return true;
}

bool ReadCameraInfoPacket(TrackingCameraState &camera, const PacketHeader header, const uint8_t *data, int length, bool erroneous)
{
	if (erroneous) return false;
	if (length < CAMERA_INFO_BASE_LENGTH) return false;

	CameraStoredInfo info;
	static_assert(CAMERA_INFO_BASE_LENGTH == 52);

	uint8_t packetVersion = data[0];
	if (packetVersion != 1)
	{
		LOG(LParsing, LInfo, "Camera #%u: Received info packet with unsupported version %d!", camera.id, packetVersion);
		camera.storage.receivedInfo = true;
		return false;
	}
	info.mcuOTPVersion = data[1];
	uint8_t resv1 = data[2], resv2 = data[3];

	static_assert(sizeof(VersionDesc) == 4);
	static_assert(sizeof(HardwareSerial) == 12);
	memcpy(&info.sbcFWVersion.num, &data[4], 4);
	memcpy(&info.mcuFWVersion.num, &data[8], 4);
	memcpy(&info.hardwareSerial.serial, &data[12], 12);
	memcpy(&info.mcuUniqueID, &data[24], 12);
	memcpy(&info.sbcRevisionCode, &data[36], 4);
	memcpy(&info.sbcSerialNumber, &data[40], 4);

	int sbcFWDesc = (data[44] << 8) | data[45];
	int mcuFWDesc = (data[46] << 8) | data[47];
	int mcuHWDesc = (data[48] << 8) | data[49];
	int mcuSubparts = ((data[50] << 8) | data[51]);
	if (length < CAMERA_INFO_BASE_LENGTH + sbcFWDesc + mcuFWDesc + mcuHWDesc + mcuSubparts)
	{
		LOG(LParsing, LError, "Camera #%u: Failed to parse info packet of size %d, expected %d + %d + %d + %d + %d!",
			camera.id, length, CAMERA_INFO_BASE_LENGTH, sbcFWDesc, mcuFWDesc, mcuHWDesc, mcuSubparts);
		return false;
	}

	const uint8_t *ptr = data + CAMERA_INFO_BASE_LENGTH;
	info.sbcFWDescriptor.resize(sbcFWDesc);
	memcpy(info.sbcFWDescriptor.data(), ptr, sbcFWDesc);
	ptr += sbcFWDesc;
	info.mcuFWDescriptor.resize(mcuFWDesc);
	memcpy(info.mcuFWDescriptor.data(), ptr, mcuFWDesc);
	ptr += mcuFWDesc;
	info.mcuHWDescriptor.resize(mcuHWDesc);
	memcpy(info.mcuHWDescriptor.data(), ptr, mcuHWDesc);
	ptr += mcuHWDesc;
	info.subpartSerials.resize(mcuSubparts / sizeof(uint64_t));
	memcpy(info.subpartSerials.data(), ptr, mcuSubparts % sizeof(uint64_t));
	ptr += mcuSubparts;

	if ((info.sbcRevisionCode >> 24) == 0xFF)
	{
		const char *modelTable[] = {
			// See https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#raspberry-pi-revision-codes
			"A",
			"B",
			"A+",
			"B+",
			"2B",
			"ALPHA",
			"CM1",
			"INVALID",
			"3B",
			"Zero",
			"CM3",
			"INVALID",
			"Zero W",
			"3B+",
			"3A+",
			"INTERNAL",
			"CM3+",
			"4B",
			"Zero 2 W",
			"400",
			"CM4",
			"CM4S",
			"INTERNAL",
			"5",
			"CM5",
			"500/500+",
			"CM5 Lite",
			"CM0",
		};
		info.sbcHWDescriptor = "Raspberry Pi ";
		if ((info.sbcRevisionCode >> 23) & 0x1)
		{ // New-Style revision
			uint16_t model = (info.sbcRevisionCode >> 4) & 0xFF;
			if (model < sizeof(modelTable)/sizeof(char*))
				info.sbcHWDescriptor += std::string(modelTable[model]);
			else
				info.sbcHWDescriptor += "INVALID MODEL";
			uint8_t rev = info.sbcRevisionCode & 0xF;
			if (rev < 9)
				info.sbcHWDescriptor += std::string(" Rev 1.") + (char)('0'+rev);
			else
				info.sbcHWDescriptor += " INVALID REV";
		}
		else
			info.sbcHWDescriptor += "OLD MODEL";
	}
	else
	{
		info.sbcHWDescriptor = asprintf_s("Unknown SBC %.2x", info.sbcRevisionCode >> 24);
	}

	if (info.mcuHWDescriptor.size() > 0)
	{
		std::string descPart;
		std::stringstream stream(info.mcuHWDescriptor);
		while (std::getline(stream, descPart, HW_DESC_SEP))
			info.mcuHWDescriptorParts.push_back(std::move(descPart));
	}

	LOG(LParsing, LInfo, "Camera #%u sent information:", camera.id);
	for (std::string &str : CameraDescribeInfo(info))
		LOG(LParsing, LInfo, "    %s", str.c_str());

	camera.storage.info = std::move(info);
	camera.storage.receivedInfo = true;
	return true;
}

void ReadCameraPacket(TrackingCameraState &camera, const PacketHeader header, const uint8_t *data, int length, bool erroneous)
{
	switch (header.tag)
	{
    case PACKET_ERROR:
    {
		ReadErrorPacket(camera, header, data, length, erroneous);
        break;
    }
	case PACKET_MODE:
	{
		ReadModePacket(camera, header, data, length, erroneous);
		break;
	}
	case PACKET_VISUAL:
	{
		ReadVisualPacket(camera, header, data, length, erroneous);
		break;
	}
	case PACKET_IMAGE:
	{
		ReadFramePacket(camera, header, data, length, erroneous);
		break;
	}
	case PACKET_STAT:
	{
		ReadStatPacket(camera, header, data, length, erroneous);
		break;
	}
	case PACKET_WIRELESS:
	{
		ReadWirelessPacket(camera, header, data, length, erroneous);
		break;
	}
	case PACKET_BGTILES:
	{
		ReadBGTilesPacket(camera, header, data, length, erroneous);
		break;
	}
	case PACKET_FW_STATUS:
	{
		LOG(LFirmwareUpdate, LTrace, "Camera %d received a firmware status packet!", camera.id);
		if (erroneous) break;
		if (length < FIRMWARE_PACKET_HEADER) break;
		if (!camera.firmware) break;
		auto camStatus = camera.firmware->contextualLock();
		camStatus->packets.emplace_back(data, data+length);
		break;
	}
	case PACKET_CAMERA_INFO:
	{
		ReadCameraInfoPacket(camera, header, data, length, erroneous);
		break;
	}
	default:
		break;
	}
}