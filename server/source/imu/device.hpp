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

#include "util/synchronised.hpp"


enum IMUDriver : uint32_t
{ // NOTE: These values should not changed as they are written to store/imu_config.json
	IMU_DRIVER_RECORD = 0,
	IMU_DRIVER_ASTERTRACK = 1,
	IMU_DRIVER_SLIMEVR = 2,
	IMU_DRIVER_REMOTE = 3
};

enum IMUDeviceProviderStatus
{
	IMU_STATUS_NORMAL = 0,
	IMU_STATUS_DISCONNECTED,
};

/**
 * An abstract IMU device class that driver-specific IMU devices derive from
 */
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

protected:
	IMUDevice(IMUIdent id, bool hasMag, bool isFused)
		: IMU(id, hasMag, isFused) {}
	IMUDevice(IMUIdent id, bool hasMag, bool isFused, IMUTracker tracker)
		: IMU(id, hasMag, isFused, tracker) {}
};

// TODO: Add separate class to derive from to passthrough IO, e.g. buttons

/**
 * An abstract IMU device provider class that driver-specific IMU providers derive from
 * May represent a static driver singleton or a concrete hardware device providing access to one or more IMU devices
 */
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

static Synchronised<std::vector<IMUConfig>> imuConfigs;

static IMUTracker getIMUTracker(const IMUIdent &id)
{
	for (const IMUConfig &config : *imuConfigs.contextualRLock())
		if (id == config.id) return config.tracker;
	return IMUTracker();
}

/* Driver-specific implementations to detect/initialise drivers and their IMU device providers */

/* Driver for AsterTrack IMU Hardware - essentially SlimeVR Firmware modified to support timesync and as such timestamps. */
bool detectAsterTrackReceivers(std::vector<std::shared_ptr<IMUDeviceProvider>> &providers);

/* Driver for standard SlimeVR IMU Hardware. Of little practical use as they do not support accurate timestamps. */
bool detectSlimeVRReceivers(std::vector<std::shared_ptr<IMUDeviceProvider>> &providers);

/**
 * Remote IMU Driver for IMUs provided by any IO subsystem that does NOT require proactively connecting to
 * E.g. An IMU that gets connected automatically while outputting the tracking stream of a tracker
 */
bool initialiseRemoteIMUs(std::vector<std::shared_ptr<IMUDeviceProvider>> &providers);
std::shared_ptr<IMUDevice> registerRemoteIMU(std::string path);
void removeRemoteIMU(std::shared_ptr<IMUDevice> remoteIMU);
IMUDeviceProvider *getRemoteIMUProvider(); // Remote IMU Driver Singleton

#endif // IMU_DEVICE_H