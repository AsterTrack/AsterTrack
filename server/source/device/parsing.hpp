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

#ifndef PARSING_H
#define PARSING_H

#include "comm/protocol_packet.hpp"

#include "pipeline/record.hpp"

#include "util/util.hpp"

struct ServerState;
struct TrackingControllerState;
struct TrackingCameraState;

// Controller packets
bool ReadSOFPacket(TrackingControllerState &controller, uint8_t *data, int length, TimePoint_t receiveTime);
bool ReadDebugPacket(TrackingControllerState &controller, uint8_t *data, int length);
bool ReadEventPacket(TrackingControllerState &controller, uint8_t *data, int length);
bool ReadStatusPacket(ServerState &state, TrackingControllerState &controller, uint8_t *data, int length);

// Stream camera packets
CameraFrameRecord ReadStreamingPacket(TrackingCameraState &camera, PacketBlocks &packet);

// Normal camera packets
bool ReadErrorPacket(TrackingCameraState &camera, PacketBlocks &packet);
bool ReadModePacket(TrackingCameraState &camera, PacketBlocks &packet);
bool ReadVisualPacket(TrackingCameraState &camera, PacketBlocks &packet);
bool ReadFramePacket(TrackingCameraState &camera, PacketBlocks &packet);
bool ReadStatPacket(TrackingCameraState &camera, PacketBlocks &packet);
bool ReadWirelessPacket(TrackingCameraState &camera, PacketBlocks &packet);
bool ReadBGTilesPacket(TrackingCameraState &camera, PacketBlocks &packet);
// Delegate to correct parsing function
void ReadCameraPacket(TrackingCameraState &camera, PacketBlocks &packet);

std::shared_ptr<CameraImage> decompressCameraImageRecord(std::shared_ptr<CameraImageRecord> &imageRecord);

#endif // PARSING_H
