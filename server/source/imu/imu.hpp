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

/**
 * Identification of an IMU across sessions
 * If required by the driver, it should also be sufficient for accessing the IMU
 * As such, serial numbers or paths are preferrable over hashes
 */
struct IMUIdent
{
	IMUDriver driver;
	std::string string;

	std::string &serial() { return string; }
	std::string &path() { return string; }

	bool operator==(const IMUIdent &other) const
	{
		return driver == other.driver && string.compare(other.string) == 0;
	}

	operator bool() const { return driver != 0; }
};

/**
 * Spacial calibration of IMU to tracker
 */
struct IMUCalib
{
	// Conversion of coordinate axes from source to ours (e.g. to change handedness)
	Eigen::Matrix<int8_t,3,3> conversion;
	int timestampOffsetUS;

	// Calibrated offset from IMU to tracker
	Eigen::Quaternionf orientation;
	Eigen::Vector3f offset;

	IMUCalib() : timestampOffsetUS(0) { conversion.setIdentity(); orientation.coeffs().setConstant(NAN); offset.setConstant(NAN); }
};

/**
 * An abstract, hardware-agnostic IMU as used by the pipeline subsystem
 */
class IMU
{
public:
	IMUIdent id;

	// Following is specific to the current session only:
	int index; // Index in pipeline subsystem, does not change during session
	bool hasMag;
	bool isFused;

	// Samples queue used depends on isFused
	BlockedQueue<IMUSampleFused, 16384> samplesFused;
	BlockedQueue<IMUSampleRaw, 16384> samplesRaw;

	IMU() : id(), index(-1) {}

	IMU(IMUIdent id, bool hasMag, bool isFused) :
		id(std::move(id)), index(-1), hasMag(hasMag), isFused(isFused) {}

	virtual ~IMU() = default;
};

/**
 * An IMU record as used by the pipeline subsystem
 */
class IMURecord : public IMU
{
public:
	~IMURecord() = default;

	IMURecord() : IMU() {}
	IMURecord(const IMU &other) :
		IMU(other.id, other.hasMag, other.isFused) {}
};

#endif // IMU_H