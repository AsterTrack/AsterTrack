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

#ifndef VPU_FIND_ACTIVE_TILES_H
#define VPU_FIND_ACTIVE_TILES_H

#include <cstdint>

typedef uint16_t VPU_MASK_INDEX;
const int VPU_INDEX_TILES = 16;
const VPU_MASK_INDEX VPU_INDEX_TERM = 0xFFFF;

static inline uint32_t getVPUMaskIndexBufferSize(uint32_t bitmaskSize)
{
	return (bitmaskSize/32/VPU_INDEX_TILES + 1) * sizeof(VPU_MASK_INDEX);
}
#endif // VPU_FIND_ACTIVE_TILES_H