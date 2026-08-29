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

#ifndef FRAMESYNC_H
#define FRAMESYNC_H

#include "util/util.hpp"
#include "util/stats.hpp"

#include <mutex>
#include <queue>

struct FrameSync
{
	std::queue<std::pair<uint8_t, TimePoint_t>> frameSOFs;
	StatDistf SOF2RecvDelay; // Models delay from trigger to receiving the frame through V4L2
	std::mutex access;
};

extern FrameSync framesync;

uint32_t correctFromFrameSync(FrameSync &framesync, TimePoint_t frameRecv, uint32_t frameID, uint32_t obsAdvance, uint32_t expAdvance, bool likelyDropped);

#endif // FRAMESYNC_H