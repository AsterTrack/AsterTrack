// //////////////////////////////////////////////////////////
// crc32.cpp
// Copyright (c) 2014,2015 Stephan Brumme. All rights reserved.
// see http://create.stephan-brumme.com/disclaimer.html
//

#include "crc32.hpp"

// big endian architectures need #define __BYTE_ORDER __BIG_ENDIAN
#ifndef _MSC_VER
#include <endian.h>
#endif

#include <array>


/// same as reset()
CRC32::CRC32()
{
	reset();
}


/// restart
void CRC32::reset()
{
	m_hash = 0;
}


namespace
{
	const uint32_t CRC32_POLY = 0xEDB88320;

	static constexpr inline uint32_t swap(uint32_t x)
	{
#if defined(__GNUC__) || defined(__clang__)
		return __builtin_bswap32(x);
#endif
#ifdef MSC_VER
		return _byteswap_ulong(x);
#endif

		return (x >> 24) |
					((x >>  8) & 0x0000FF00) |
					((x <<  8) & 0x00FF0000) |
					 (x << 24);
	}

	static constexpr void SliceByEight(std::array<std::array<uint32_t,256>,8> &LUT)
	{
		// slicing-by-8 algorithm (from Intel):
		// http://www.intel.com/technology/comms/perfnet/download/CRC_generators.pdf
		// http://sourceforge.net/projects/slicing-by-8/
		for (unsigned int i = 0; i <= 0xFF; i++)
		{
			LUT[1][i] = (LUT[0][i] >> 8) ^ LUT[0][LUT[0][i] & 0xFF];
			LUT[2][i] = (LUT[1][i] >> 8) ^ LUT[0][LUT[1][i] & 0xFF];
			LUT[3][i] = (LUT[2][i] >> 8) ^ LUT[0][LUT[2][i] & 0xFF];

			LUT[4][i] = (LUT[3][i] >> 8) ^ LUT[0][LUT[3][i] & 0xFF];
			LUT[5][i] = (LUT[4][i] >> 8) ^ LUT[0][LUT[4][i] & 0xFF];
			LUT[6][i] = (LUT[5][i] >> 8) ^ LUT[0][LUT[5][i] & 0xFF];
			LUT[7][i] = (LUT[6][i] >> 8) ^ LUT[0][LUT[6][i] & 0xFF];
		}
	}

	constexpr auto crc32Lookup{[]() constexpr{
		std::array<std::array<uint32_t,256>,8> LUT{};
		for (uint32_t i = 0; i <= 0xFF; i++)
		{
			uint32_t crc = i;
			for (unsigned int j = 0; j < 8; j++)
				crc = (crc >> 1) ^ ((crc & 1) * CRC32_POLY);
			LUT[0][i] = crc;
		}
		SliceByEight(LUT);
		return LUT;
	}()};
}


/// add arbitrary number of bytes
void CRC32::add(const void* data, size_t numBytes)
{
	uint32_t* current = (uint32_t*) data;
	uint32_t crc = ~m_hash;

	// process eight bytes at once
	while (numBytes >= 8)
	{
#if defined(__BYTE_ORDER) && (__BYTE_ORDER != 0) && (__BYTE_ORDER == __BIG_ENDIAN)
		uint32_t one = *current++ ^ swap(crc);
		uint32_t two = *current++;
		crc =	crc32Lookup[7][ one>>24        ] ^
				crc32Lookup[6][(one>>16) & 0xFF] ^
				crc32Lookup[5][(one>> 8) & 0xFF] ^
				crc32Lookup[4][ one      & 0xFF] ^
				crc32Lookup[3][ two>>24        ] ^
				crc32Lookup[2][(two>>16) & 0xFF] ^
				crc32Lookup[1][(two>> 8) & 0xFF] ^
				crc32Lookup[0][ two      & 0xFF];
#else
		uint32_t one = *current++ ^ crc;
		uint32_t two = *current++;
		crc =	crc32Lookup[7][ one      & 0xFF] ^
				crc32Lookup[6][(one>> 8) & 0xFF] ^
				crc32Lookup[5][(one>>16) & 0xFF] ^
				crc32Lookup[4][ one>>24        ] ^
				crc32Lookup[3][ two      & 0xFF] ^
				crc32Lookup[2][(two>> 8) & 0xFF] ^
				crc32Lookup[1][(two>>16) & 0xFF] ^
				crc32Lookup[0][ two>>24        ];
#endif
		numBytes -= 8;
	}

	unsigned char* currentChar = (unsigned char*) current;
	// remaining 1 to 7 bytes (standard CRC table-based algorithm)
	while (numBytes--)
		crc = (crc >> 8) ^ crc32Lookup[0][(crc & 0xFF) ^ *currentChar++];

	m_hash = ~crc;
}


/// return latest hash as 8 hex characters
std::string CRC32::getHash()
{
	// convert hash to string
	static const char dec2hex[16+1] = "0123456789abcdef";

	char hashBuffer[8+1];

	hashBuffer[0] = dec2hex[ m_hash >> 28      ];
	hashBuffer[1] = dec2hex[(m_hash >> 24) & 15];
	hashBuffer[2] = dec2hex[(m_hash >> 20) & 15];
	hashBuffer[3] = dec2hex[(m_hash >> 16) & 15];
	hashBuffer[4] = dec2hex[(m_hash >> 12) & 15];
	hashBuffer[5] = dec2hex[(m_hash >>  8) & 15];
	hashBuffer[6] = dec2hex[(m_hash >>  4) & 15];
	hashBuffer[7] = dec2hex[ m_hash        & 15];
	// zero-terminated string
	hashBuffer[8] = 0;

	// convert to std::string
	return hashBuffer;
}


/// return latest hash as bytes
void CRC32::getHash(unsigned char buffer[CRC32::HashBytes])
{
	buffer[0] = (m_hash >> 24) & 0xFF;
	buffer[1] = (m_hash >> 16) & 0xFF;
	buffer[2] = (m_hash >>  8) & 0xFF;
	buffer[3] =  m_hash        & 0xFF;
}


/// compute CRC32 of a memory block
std::string CRC32::operator()(const void* data, size_t numBytes)
{
	reset();
	add(data, numBytes);
	return getHash();
}


/// compute CRC32 of a string, excluding final zero
std::string CRC32::operator()(const std::string& text)
{
	reset();
	add(text.c_str(), text.size());
	return getHash();
}
