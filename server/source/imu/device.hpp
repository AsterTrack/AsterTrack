/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#ifndef IMU_DEVICE_H
#define IMU_DEVICE_H

#include "imu/imu.hpp"

#include "comm/timesync.hpp"

enum IMUDriver : uint32_t
{
	IMU_DRIVER_RECORD = 0,
	IMU_DRIVER_ASTERTRACK,
	IMU_DRIVER_SLIMEVR,
	IMU_DRIVER_REMOTE
};

enum IMUDeviceProviderStatus
{
	IMU_STATUS_NORMAL = 0,
	IMU_STATUS_DISCONNECTED,
};

class IMUDevice : public IMU
{
public:
	// State
	bool hasBattery, isPlugged;
	float batteryLevel;
	float batteryVolts;
	float temperature;
	float signalStrength;

	virtual ~IMUDevice() = default;

	// TODO: Methods to setup passthrough to IO, e.g. buttons to VRPN

protected:
	IMUDevice(bool hasMag, bool isFused)
		: IMU(hasMag, isFused) {}
	IMUDevice(bool hasMag, bool isFused, IMUDriver driver, int provider, int device)
		: IMU(hasMag, isFused, driver, provider, device) {}
};

class IMUDeviceProvider
{
public:
	IMUDriver driver;

	std::vector<std::shared_ptr<IMUDevice>> devices;

	// Can be optionally used to provide debug data about timing
	bool recordTimeSyncMeasurements = false;
	BlockedQueue<TimeSyncMeasurement, 4096> timeSyncMeasurements;
	bool recordLatencyMeasurements = false;
	LatencyDescriptor latencyDescriptions;
	BlockedQueue<LatencyMeasurement, 4096> latencyMeasurements;

	virtual ~IMUDeviceProvider() = default;

	virtual IMUDeviceProviderStatus poll(int &updatedDevices, int &changedDevices) = 0;

protected:
	IMUDeviceProvider(IMUDriver driver) : driver(driver) {}
};

bool detectAsterTrackReceivers(std::vector<std::shared_ptr<IMUDeviceProvider>> &providers);
bool detectSlimeVRReceivers(std::vector<std::shared_ptr<IMUDeviceProvider>> &providers);
bool initialiseRemoteIMUs(std::vector<std::shared_ptr<IMUDeviceProvider>> &providers);

std::shared_ptr<IMUDevice> registerRemoteIMU(int deviceID);
IMUDeviceProvider *getRemoteIMUProvider();

#endif // IMU_DEVICE_H