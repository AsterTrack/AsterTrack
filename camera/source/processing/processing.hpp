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

#ifndef PROCESSING_H
#define PROCESSING_H

#include "videocore/vc_base.h"
#include "comm/packet.hpp"
#include "util/util.hpp"

struct FrameBuffer
{
	void* header = nullptr;
	uint8_t* memory = nullptr;
	uint16_t stride;
	VC_BUFFER *bitmsk = nullptr;
	uint32_t ID = 0;
	TimePoint_t SOF, rcv;
	uint32_t time_qpu;
	int skippedTrigger, skippedCPU, skippedQPU;

	~FrameBuffer();
};

struct TrackingCameraState;
struct ImageStreamState;

// Defined in main.cpp
bool handleError(ErrorTag error, bool serious = true, const char* reportBuf = nullptr, int reportLen = 0);
bool handleConsoleInputStreaming();
bool getFramesyncActive();
void sendWirelessStatusPacket(TrackingCameraState &state);
StatPacket prepareSystemStatusPacket(TrackingCameraState &state, long deltaStatUS);
bool prepareImageStreamingPacket(const FrameBuffer &frame, ImageStreamState stream);
int getInterleavedQueuedBytes();
int sendInterleavedQueuedPackets(int commTimeUS);

// Defined in processing.cpp
bool ProcessingStage(TrackingCameraState &state, VC_BASE &base);

#endif // PROCESSING_H