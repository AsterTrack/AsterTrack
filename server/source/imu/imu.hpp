/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#ifndef IMU_H
#define IMU_H

#include "util/util.hpp"
#include "util/eigendef.hpp"
#include "util/blocked_vector.hpp"

enum IMUDriver : uint32_t;

struct IMUSampleFused
{
	TimePoint_t timestamp;
	Eigen::Quaternionf quat;
	Eigen::Vector3f accel;
};

struct IMUSampleRaw
{
	TimePoint_t timestamp;
	Eigen::Vector3f gyro;
	Eigen::Vector3f accel;
	Eigen::Vector3f mag;
};

bool inline operator<(const IMUSampleFused& imu, const TimePoint_t& ts) { return imu.timestamp < ts; }
bool inline operator<(const TimePoint_t& ts, const IMUSampleFused& imu) { return ts < imu.timestamp; }
bool inline operator<(const IMUSampleRaw& imu, const TimePoint_t& ts) { return imu.timestamp < ts; }
bool inline operator<(const TimePoint_t& ts, const IMUSampleRaw& imu) { return ts < imu.timestamp; }

class IMU
{
public:
	int trackerID, index;

	// Identification
	IMUDriver driver;
	int provider, device; // Driver defined

	bool hasMag;
	bool isFused;

	BlockedQueue<IMUSampleFused, 16384> samplesFused;
	BlockedQueue<IMUSampleRaw, 16384> samplesRaw;

	IMU(bool hasMag, bool isFused) :
		trackerID(0), index(-1),
		driver((IMUDriver)0), provider(0), device(0),
		hasMag(hasMag), isFused(isFused) {}
	IMU(bool hasMag, bool isFused, IMUDriver driver, int provider, int device) :
		trackerID(0), index(-1),
		driver(driver), provider(provider), device(device),
		hasMag(hasMag), isFused(isFused) {}

	virtual ~IMU() = default;
};

class IMURecord : public IMU
{
public:
	~IMURecord() = default;
	IMURecord() : IMU(false, false) {}
	IMURecord(const IMU &other) :
		IMU(other.hasMag, other.isFused,
			other.driver, other.provider, other.device)
	{
		trackerID = other.trackerID;
	}
};

#endif // IMU_H