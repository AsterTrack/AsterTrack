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

static FirmwareTag parseFirmwareTag(const uint8_t *dataTail, uint32_t dataLen)
{
	FirmwareTag tag = {};

	if (dataLen < sizeof(FIRMWARE_TAG_KEY)+FIRMWARE_TAG_HEADER_MIN)
		return tag; // Too small to hold a tag.

	const uint8_t *header = dataTail-sizeof(FIRMWARE_TAG_KEY);
	if (memcmp(header, FIRMWARE_TAG_KEY, sizeof(FIRMWARE_TAG_KEY)))
		return tag; // Not tagged.

	int version = tag.tagVersion = header[-1];
	if (version == 1)
		tag.size = 10 + sizeof(FIRMWARE_TAG_KEY);
	else if (version == 2)
		tag.size = 12 + sizeof(FIRMWARE_TAG_KEY);
	else
		return tag; // Unsupported version.

	if (dataLen < tag.size)
		return tag; // Too small to hold specified tag.

	tag.valid = true;
	tag.type = (FirmwareTagType)header[-2];
	// 2 reserved bytes
	memcpy(&tag.version.num, &header[-8], 4);
	tag.descLen = *(uint16_t*)&header[-10];
	if (version >= 2)
		tag.compatLen = *(uint16_t*)&header[-12];
	// May expand here

	return tag;
}

static FirmwareTag ReadFirmwareTag(const uint8_t *data, uint32_t dataLen)
{
	FirmwareTag tag = parseFirmwareTag(data+dataLen, dataLen);
	if (!tag.valid)
		return tag; // Not tagged or tag is invalid.
	if (dataLen <= tag.size+tag.descLen+tag.compatLen)
		return tag; // Tag is missing data, undesired.

	if (tag.descLen > 0)
		tag.descStr = std::string((char*)data+dataLen-tag.size-tag.descLen, tag.descLen);

	if (tag.compatLen > 0)
		tag.compatStr = std::string((char*)data+dataLen-tag.size-tag.descLen-tag.compatLen, tag.compatLen);

	return tag;
}

static FirmwareTag ReadFirmwareTag(std::ifstream &fs)
{
	FirmwareTag tag = {};
	
	fs.seekg(0, std::ios::end);
	int size = fs.tellg();
	if (size < sizeof(FIRMWARE_TAG_KEY)+FIRMWARE_TAG_HEADER_MIN) return tag;

	uint8_t tagBuffer[sizeof(FIRMWARE_TAG_KEY)+FIRMWARE_TAG_HEADER_MAX];
	int header = std::min<int>(size, sizeof(tagBuffer));
	fs.seekg(size - header);
	if (fs.fail()) return tag;
	fs.read((char*)tagBuffer, header);
	if (fs.fail()) return tag;

	tag = parseFirmwareTag(tagBuffer+header, header);
	if (!tag.valid)
		return tag; // Not tagged or tag is invalid.
	if (size <= tag.size+tag.descLen+tag.compatLen)
		return tag; // Tag is missing data, undesired.

	if (tag.descLen > 0)
	{
		fs.seekg(-tag.size-tag.descLen, std::ios::end);
		if (fs.fail()) return tag;

		tag.descStr.resize(tag.descLen);
		fs.read(tag.descStr.data(), tag.descLen);
		if (fs.fail()) tag.descStr.clear();
	}

	if (tag.compatLen > 0)
	{
		fs.seekg(-tag.size-tag.descLen-tag.compatLen, std::ios::end);
		if (fs.fail()) return tag;

		tag.compatStr.resize(tag.compatLen);
		fs.read(tag.compatStr.data(), tag.compatLen);
		if (fs.fail()) tag.compatStr.clear();
	}

	return tag;
}