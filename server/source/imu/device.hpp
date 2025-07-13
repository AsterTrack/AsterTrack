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

#include "comm/timingRecord.hpp" // TimingRecord


enum IMUDriver : uint32_t
{ // NOTE: These values should not changed as they are written to store/imu_config.json
	IMU_DRIVER_NONE = 0,
	IMU_DRIVER_ASTERTRACK = 1,
	IMU_DRIVER_SLIMEVR = 2,
	IMU_DRIVER_REMOTE = 3,
	IMU_DRIVER_MAX
};

enum IMUDeviceProviderStatus
{
	IMU_STATUS_NO_DEVICES = 0,
	IMU_STATUS_DEVICES_CONNECTED = 0,
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

	TimingRecord timingRecord;

	virtual ~IMUDevice() = default;

protected:
	IMUDevice(IMUIdent id, bool hasMag, bool isFused)
		: IMU(id, hasMag, isFused) {}
};
typedef std::vector<std::shared_ptr<IMUDevice>> IMUDeviceList;

// TODO: Add separate class to derive from to passthrough IO, e.g. buttons

/**
 * An abstract IMU device provider class that driver-specific IMU providers derive from
 * May represent a static driver singleton or a concrete hardware device providing access to one or more IMU devices
 */
class IMUDeviceProvider
{
public:
	IMUDriver driver;

	TimingRecord timingRecord;

	virtual ~IMUDeviceProvider() = default;

	virtual IMUDeviceProviderStatus poll(int &updated, IMUDeviceList &removed, IMUDeviceList &added) = 0;

protected:
	IMUDeviceProvider(IMUDriver driver) : driver(driver) {}
};
typedef std::vector<std::shared_ptr<IMUDeviceProvider>> IMUDeviceProviderList;

/* Driver-specific implementations to detect/initialise drivers and their IMU device providers */

/* Driver for AsterTrack IMU Hardware - essentially SlimeVR Firmware modified to support timesync and as such timestamps. */
bool detectAsterTrackReceivers(IMUDeviceProviderList &providers);

/* Driver for standard SlimeVR IMU Hardware. Of little practical use as they do not support accurate timestamps. */
bool detectSlimeVRReceivers(IMUDeviceProviderList &providers);

/**
 * Remote IMU Driver for IMUs provided by any IO subsystem that does NOT require proactively connecting to
 * E.g. An IMU that gets connected automatically while outputting the tracking stream of a tracker
 */
bool initialiseRemoteIMUs(IMUDeviceProviderList &providers);
std::shared_ptr<IMUDevice> registerRemoteIMU(std::string path);
void removeRemoteIMU(std::shared_ptr<IMUDevice> remoteIMU);
IMUDeviceProvider *getRemoteIMUProvider(); // Remote IMU Driver Singleton

#endif // IMU_DEVICE_H