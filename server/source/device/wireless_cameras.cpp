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

#include "wireless_cameras.hpp"
#include "server/server.hpp"

#include "comm/wireless_server_client.hpp"

#include "ui/shared.hpp" // Signals to UI

#include "device/tracking_camera.hpp"
#include "device/parsing.hpp"

#include "util/log.hpp"
#include "util/util.hpp" // printBuffer, TimePoint_t

#include <chrono>
#include <memory>

void StartWirelessServer(ServerCommState &server, ServerState &state)
{
	if (server.thread)
		return;
	LOG(LServer, LInfo, "Starting network server!\n");
	// Init server
	if (server.portSet.empty())
		server.portSet = "45732";
	server.portUsed = server.portSet;
	server.socket = WirelessServerOpen(server.portUsed);
	if (server.socket == 0)
	{
		LOG(LServer, LError, "Could not start network server!\n");
		return;
	}
	server.callbacks.userData1 = &state;
	server.callbacks.onIdentify = [](ClientCommState &client) { // Find or setup camera
		ServerState &state = *((ServerState*)client.callbacks.userData1);
		LOG(LServer, LInfo, "Established server connection to camera #%u!\n", client.otherIdent.id);
		std::unique_lock dev_lock(state.deviceAccessMutex, std::chrono::milliseconds(50));
		if (!dev_lock.owns_lock())
		{ // Likely StopDeviceMode waiting on us to stop thread
			if (state.mode != MODE_Device)
				LOG(LServer, LDarn, "Aborted wireless camera connection because we're exiting device mode!\n");
			else
			 	LOG(LServer, LError, "Aborted wireless camera connection because of failure to aquire lock for unknown reason!\n");
			return false;
		}
		// Setup camera with server connection
		std::shared_ptr<TrackingCameraState> camera = EnsureCamera(state, client.otherIdent.id);
		if (camera->client)
		{ // Already had a wireless client connection, perhaps camera crashed and didn't terminate the last connection properly
			camera->client->thread->request_stop();
			LOG(LServer, LWarn, "Camera #%u was already wirelessly connected, maybe had a stale connection?", camera->id);
		}
		camera->client = &client;
		client.callbacks.userData2 = camera;
		{
			auto camState = camera->state.contextualLock();
			camState->hadServerConnected = true;
			// Clear any possible error state
			if (camState->error.encountered && dtMS(camState->error.time, sclock::now()) > 1000)
			{ // Might need this overlap protection to prevent race-conditions
				camState->error.recoverTime = sclock::now();
				camState->error.recovered = true;
				camState->error.encountered = false;
			}
		}
		// Setup stream subsystem
		if (!camera->sync)
		{ // Init with internal sync if no controller is known to be connected
			SetCameraSyncNone(*state.stream.contextualLock(), camera, 1000.0f / 144);
		}
		SignalServerEvent(EVT_UPDATE_CAMERAS);
		return true;
	};
	server.callbacks.onDisconnect = [](ClientCommState &client) { // Remove camera
		ServerState &state = *((ServerState*)client.callbacks.userData1);
		if (client.callbacks.userData2)
		{ // Camera was already identified
			if (client.callbacks.userData2.use_count() == 1)
				LOG(LServer, LWarn, "Disconnecting camera was already erased from devices!\n");
			TrackingCameraState &camera = *std::static_pointer_cast<TrackingCameraState>(client.callbacks.userData2);
			if (camera.client != &client)
			{
				LOG(LServer, LWarn, "Stale client connection thread of camera #%u disconnected!\n", camera.id);
				return;
			}
			{
				auto state = camera.state.contextualLock();
				state->lastWirelessConnection = sclock::now();
			}
			if (client.callbacks.userData2.use_count() == 1)
				LOG(LServer, LWarn, "Disconnecting camera was already erased from devices!\n");
			camera.client = NULL;
			LOG(LServer, LInfo, "Camera #%u lost server connection!\n", camera.id);
			// If camera is now unreachable, DeviceSupervisorThread will wait for it to reconnect before removing it entirely
			client.callbacks.userData2 = nullptr;
		}
		else
		{
			LOG(LServer, LWarn, "Lost connection to camera before it could be identified!\n");
		}
	};
	server.callbacks.onReceivePacketHeader = [](ClientCommState &client, PacketHeader &header, TimePoint_t receiveTime)
	{
		if (!header.isStreamPacket() && header.tag != PACKET_FRAME_SIGNAL)
			return true;
		ServerState &state = *((ServerState*)client.callbacks.userData1);
		std::shared_lock dev_lock(state.deviceAccessMutex, std::chrono::milliseconds(10));
		if (!dev_lock.owns_lock()) return false; // Likely StopDeviceMode waiting on us to stop thread
		assert(client.callbacks.userData2);
		TrackingCameraState &camera = *std::static_pointer_cast<TrackingCameraState>(client.callbacks.userData2);
		if (!camera.sync)
		{
			LOG(LStreaming, LTrace, "Wireless Camera %u send a streaming packet but was not set up for streaming!\n", camera.id);
			return false;
		}
		if (header.tag == PACKET_FRAME_SIGNAL)
		{ // Frame is starting to be processed
			auto sync_lock = camera.sync->contextualLock();
			if (sync_lock->source == SYNC_NONE)
			{ // No sync, this is only source for SOF timing, estimate actual SOF
				// Update estimate of SOF (assuming regular SOFs)
				TimePoint_t timeSOF = receiveTime - std::chrono::microseconds(8000);
				//timeSOF = UpdateSOFPredictions(*camera.sync, header.frameID, timeSOF);
				FrameID frameID = EstimateFullFrameID(*sync_lock, header.frameID);
				RegisterSOF(*sync_lock, frameID, timeSOF);
			}
			auto frame = RegisterCameraFrame(*sync_lock, camera.syncIndex, header.frameID);
			if (frame)
				LOG(LStreaming, LDebug, "Wireless Camera %u announced packet for frame %d (%d)!\n", camera.id, frame->ID, header.frameID);
			else
				LOG(LStreaming, LDebug, "Wireless Camera %u announced packet for non-existant frame with ID %d!\n", camera.id, header.frameID);
			return true;
		}
		else if (header.isStreamPacket())
		{
			if (!RegisterStreamPacket(*camera.sync->contextualLock(), camera.syncIndex, header.frameID, receiveTime))
			{
				LOG(LStreaming, LTrace, "Wireless Camera %u sent a streaming packet but was not set up for streaming!\n", camera.id);
				return false;
			}
		}
		return true;
	};
	server.callbacks.onReceivePacketBlock = [](ClientCommState &client, PacketHeader &header, uint8_t *data, unsigned int len, TimePoint_t receiveTime)
	{
		ServerState &state = *((ServerState*)client.callbacks.userData1);
		std::shared_lock dev_lock(state.deviceAccessMutex, std::chrono::milliseconds(10));
		if (!dev_lock.owns_lock()) return; // Likely StopDeviceMode waiting on us to stop thread
		assert(client.callbacks.userData2);
		TrackingCameraState &camera = *std::static_pointer_cast<TrackingCameraState>(client.callbacks.userData2);
		if (!camera.sync) return; // Likely removed camera already (possible when halting during debugging)
		// Update statistics of streaming packet
		if (header.isStreamPacket())
		{
			if (!RegisterStreamBlock(*camera.sync->contextualLock(), camera.syncIndex, header.frameID))
			{
				LOG(LParsing, LError, "---- Received stream block for non-existant frame %d or unregistered stream packet!\n", header.frameID);
			}
		}
	};
	server.callbacks.onReceivePacket = [](ClientCommState &client, PacketHeader &header, uint8_t *data, unsigned int len, TimePoint_t receiveTime, bool erroneous)
	{
		assert(client.callbacks.userData2);
		ServerState &state = *((ServerState*)client.callbacks.userData1);
		TrackingCameraState &camera = *std::static_pointer_cast<TrackingCameraState>(client.callbacks.userData2);
		std::shared_lock dev_lock(state.deviceAccessMutex, std::chrono::milliseconds(10));
		if (!dev_lock.owns_lock()) return; // Likely StopDeviceMode waiting on us to stop thread
		if (header.isStreamPacket())
		{
			auto cameraFrame = ReadStreamingPacket(camera, header, data, len, erroneous);
			int blobCount = cameraFrame.rawPoints2D.size();
			auto sync_lock = camera.sync->contextualLock();
			SyncedFrame *frame = RegisterStreamPacketComplete(*sync_lock, camera.syncIndex, header.frameID, std::move(cameraFrame), erroneous);
			if (frame)
			{ // Packet existed for frame
				LOG(LStreaming, LTrace, "Camera %u (TCP) fully transmitted stream packet %d for frame %d (%d) with %d blobs!\n",
					camera.id, header.tag, frame->ID, frame->ID&0xFF, blobCount);
				if (frame->previouslyProcessed)
				{
					LOG(LStreaming, LTrace, "---- Camera %u finally transmitted stream packet for frame %d with %d blobs, %.2fms into the frame, %.2fms after processing!\n",
						camera.id, frame->ID, blobCount, dtMS(frame->SOF, sclock::now()), dtMS(frame->lastProcessed, sclock::now()));
				}
			}
			else
			{
				LOG(LParsing, LError, "---- Completed stream packet for non-existant frame %d or unregistered stream packet!\n", header.frameID);
			}
		}
		else
		{
			ReadCameraPacket(camera, header, data, len, erroneous);
		}
	};

	// Start comm thread
	server.thread = new std::jthread(WirelessServerThread, &server);
}

void StopWirelessServer(ServerCommState &server)
{
	if (!server.thread)
		return;
	LOG(LServer, LInfo, "Stopping network server!\n");
	server.portUsed.clear();
	// Join server thread, it will do cleanup
	delete server.thread;
	server.thread = NULL;
}