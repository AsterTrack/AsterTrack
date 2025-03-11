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

struct IMUSample
{
	TimePoint_t timestamp;
	Eigen::Quaternionf quat;
	Eigen::Vector3f accel;
};

bool inline operator<(const IMUSample& imu, const TimePoint_t& ts) { return imu.timestamp < ts; }
bool inline operator<(const TimePoint_t& ts, const IMUSample& imu) { return ts < imu.timestamp; }

class IMU
{
public:
	int trackerID, index;

	// Identification
	IMUDriver driver;
	int provider, device; // Driver defined

	BlockedQueue<IMUSample, 16384> samples;

	IMU() :
		driver((IMUDriver)0), provider(0), device(0),
		trackerID(0), index(-1) {}
	IMU(IMUDriver driver, int provider, int device) :
		driver(driver), provider(provider), device(device),
		trackerID(0), index(-1) {}

	virtual ~IMU() = default;
};

class IMURecord : public IMU
{
public:
	~IMURecord() = default;
	IMURecord() : IMU() {}
};

#endif // IMU_H