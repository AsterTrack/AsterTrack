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
};

/**
 * Association of an IMU to a tracker (tracked Target or Marker)
 * Includes tracker indentification and spacial calibration of tracker to IMU
 */
struct IMUTracker
{
	int id; // Associated Tracker ID - 0 is None
	float size; // Associated Marker Size - NAN is None
	Eigen::Matrix<int8_t,3,3> conversion; // Technically this is not specific to the tracker
	Eigen::Quaternionf orientation;
	Eigen::Vector3f offset;

	IMUTracker() : id(0), size(NAN) { conversion.setIdentity(); orientation.coeffs().setConstant(NAN); }
	IMUTracker(int id) : id(id), size(NAN) { conversion.setIdentity(); orientation.coeffs().setConstant(NAN); }
	IMUTracker(float size) : id(0), size(size) { conversion.setIdentity(); orientation.setIdentity(); }

	operator bool() const { return id != 0 || !std::isnan(size); }
	bool isMarker() const { return !std::isnan(size); }
};

/**
 * IMU Identification and Tracker Association as saved between sessions
 */
struct IMUConfig
{
	IMUIdent id;
	IMUTracker tracker;
};

/**
 * An abstract, hardware-agnostic IMU as used by the pipeline subsystem
 */
class IMU
{
public:
	IMUIdent id;
	IMUTracker tracker;

	// Following is specific to the current session only:
	int index; // Index in pipeline subsystem, does not change during session
	bool hasMag;
	bool isFused;

	BlockedQueue<IMUSampleFused, 16384> samplesFused;
	BlockedQueue<IMUSampleRaw, 16384> samplesRaw;

	IMU() : id(), tracker(), index(-1) {}

	IMU(IMUIdent id, bool hasMag, bool isFused) :
		id(std::move(id)), tracker(),
		index(-1), hasMag(hasMag), isFused(isFused) {}

	IMU(IMUIdent id, bool hasMag, bool isFused, IMUTracker tracker) :
		id(std::move(id)), tracker(tracker),
		index(-1), hasMag(hasMag), isFused(isFused) {}

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
		IMU(other.id, other.hasMag, other.isFused, other.tracker) {}
};

#endif // IMU_H