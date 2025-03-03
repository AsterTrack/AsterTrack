/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#ifndef IMU_DEVICE_H
#define IMU_DEVICE_H

#include "util/eigendef.hpp"

#include "comm/timesync.hpp"

#include "util/util.hpp"
#include "util/blocked_vector.hpp"

#include <deque>

enum IMUDeviceDriver
{
	IMU_DRIVER_ASTERTRACK = 1,
	IMU_DRIVER_SLIMEVR
};

enum IMUDeviceProviderStatus
{
	IMU_STATUS_NORMAL = 0,
	IMU_STATUS_DISCONNECTED,
};

struct IMUReport
{
	TimePoint_t timestamp;
	Eigen::Quaternionf quat;
	Eigen::Vector3f accel;
};

class IMUDevice
{
public:
	IMUDeviceDriver driver;
	int provider;
	int device;

	// State
	bool hasBattery, isPlugged;
	float batteryLevel;
	float batteryVolts;
	float temperature;
	float signalStrength;

	// IMU reports
	std::deque<IMUReport> reports;
	bool dirty;

    virtual ~IMUDevice() = default;

	// TODO: Methods to setup passthrough to IO, e.g. buttons to VRPN
};

class IMUDeviceProvider
{
public:
	IMUDeviceDriver driver;

	std::vector<std::shared_ptr<IMUDevice>> devices;

    virtual ~IMUDeviceProvider() = default;

	virtual IMUDeviceProviderStatus poll(int &updatedDevices, int &changedDevices) = 0;

protected:
	IMUDeviceProvider(IMUDeviceDriver driver) : driver(driver) {}
};

bool detectAsterTrackReceivers(std::vector<std::shared_ptr<IMUDeviceProvider>> &providers);
bool detectSlimeVRReceivers(std::vector<std::shared_ptr<IMUDeviceProvider>> &providers);

#endif