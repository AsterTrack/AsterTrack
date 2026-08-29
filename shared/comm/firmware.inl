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

#include "comm/packet.hpp"

#include <fstream>

static bool ReadFirmwareTag(std::ifstream &fs, FirmwareTagHeader &tag)
{
	tag.valid = false;
	
	fs.seekg(0, std::ios::end);
	int size = fs.tellg();
	if (size <= FIRMWARE_TAG_SIZE) return false;

	fs.seekg(size-FIRMWARE_TAG_SIZE);
	if (fs.fail()) return false;

	uint8_t tagBuffer[FIRMWARE_TAG_SIZE];
	fs.read((char*)tagBuffer, FIRMWARE_TAG_SIZE);
	if (fs.fail()) return false;

	if (!parseFirmwareTagHeader(tagBuffer, &tag))
		return false;
	if (size <= tag.size+tag.descLen)
		return false; // Is tag itself, undesired
	tag.valid = true;

	if (tag.descLen > 0)
	{
		fs.seekg(-tag.size-tag.descLen, std::ios::end);
		if (fs.fail()) return true;

		tag.descriptor.resize(tag.descLen);
		fs.read(tag.descriptor.data(), tag.descLen);
		if (fs.fail()) tag.descriptor.clear();
	}

	return true;
}