
#include "imu.hpp"

#include "util/log.hpp"

#include "hidapi/hidapi.h"

#include "flexkalman/EigenQuatExponentialMap.h" // quat_exp, quat_ln

enum SLIMEVR_BOARD_TYPE
{
	SLIMEVR_BOARD_UNKNOWN,
	SLIMEVR_BOARD_LEGACY,
	SLIMEVR_BOARD_DEV,
	SLIMEVR_BOARD_NODEMCU,
	SLIMEVR_BOARD_CUSTOM,
	SLIMEVR_BOARD_WROOM32,
	SLIMEVR_BOARD_WEMOSD1MINI,
	SLIMEVR_BOARD_TTGO_TBASE,
	SLIMEVR_BOARD_ESP01,
	SLIMEVR_BOARD_SLIMEVR,
	SLIMEVR_BOARD_LOLIN_C3_MINI,
	SLIMEVR_BOARD_BEETLE32C3,
	SLIMEVR_BOARD_ES32C3DEVKITM1,
	SLIMEVR_BOARD_OWOTRACK,
	SLIMEVR_BOARD_WRANGLER,
	SLIMEVR_BOARD_MOCOPI,
	SLIMEVR_BOARD_WEMOSWROOM02,
	SLIMEVR_BOARD_XIAO_ESP32C3,
	SLIMEVR_BOARD_HARITORA,
	SLIMEVR_BOARD_MAX,
	SLIMEVR_BOARD_DEV_RESERVED = 250
};

enum SLIMEVR_MCU_TYPE
{
	SLIMEVR_MCU_UNKNOWN,
	SLIMEVR_MCU_ESP8266,
	SLIMEVR_MCU_ESP32,
	SLIMEVR_MCU_OWOTRACK_ANDROID,
	SLIMEVR_MCU_WRANGLER,
	SLIMEVR_MCU_OWOTRACK_IOS,
	SLIMEVR_MCU_ESP32_C3,
	SLIMEVR_MCU_MOCOPI,
	SLIMEVR_MCU_HARITORA,
	SLIMEVR_MCU_MAX,
	SLIMEVR_MCU_DEV_RESERVED = 250
};

enum SLIMEVR_TRACKER_STATUS
{
	SLIMEVR_TRACKER_DISCONNECTED,
	SLIMEVR_TRACKER_OK,
	SLIMEVR_TRACKER_BUSY,
	SLIMEVR_TRACKER_ERROR,
	SLIMEVR_TRACKER_OCCLUDED,
	SLIMEVR_TRACKER_TIMED_OUT
};

class SlimeVRTracker : public IMUDevice
{
public:
	SLIMEVR_BOARD_TYPE board;
	SLIMEVR_MCU_TYPE MCU;
	SLIMEVR_TRACKER_STATUS status;

	~SlimeVRTracker() = default;
};

class SlimeVRReceiver : public IMUDeviceProvider
{
public:
	std::string path;
	hid_device *handle;

	SlimeVRReceiver(const char *path, hid_device *handle) : path(path), handle(handle), IMUDeviceProvider(IMU_DRIVER_SLIMEVR) {}

	~SlimeVRReceiver()
	{
		hid_close(handle);
	}

	IMUDeviceProviderStatus poll(int &updatedDevices, int &changedDevices);
	bool parseIMUPacket(SlimeVRTracker &tracker, uint8_t data[16]);
};

IMUDeviceProviderStatus SlimeVRReceiver::poll(int &updatedDevices, int &changedDevices)
{
	thread_local uint8_t data[64+8]; 
	int reports = 0;
	int read;
	LOG(LIO, LDebug, "Polling SlimeVR Receiver %s!", path.c_str());
	while ((read = hid_read(handle, data, sizeof(data))) > 0)
	{
		int packets = read/16;
		LOG(LIO, LDebug, "  SlimeVR Receiver sent %d bytes (%d packets)!", read, packets);
		for (int p = 0; p < packets; p++)
		{
			int i = p*16;
			if (data[i+0] == 255) continue;
			uint8_t trkID = data[i+1];
			if (devices.size() <= trkID)
				devices.resize(trkID+1);
			if (!devices[trkID])
			{
				devices[trkID] = std::make_shared<SlimeVRTracker>();
				LOG(LIO, LInfo, "    Registered new IMU!");
				changedDevices++;
			}
			SlimeVRTracker &tracker = *static_cast<SlimeVRTracker*>(devices[trkID].get());
			if (parseIMUPacket(tracker, data+i))
				updatedDevices++;
			/* else if (tracker.status == SLIMEVR_TRACKER_DISCONNECTED || tracker.status == SLIMEVR_TRACKER_ERROR)
				devices[trkID] = nullptr; */
		}
	}
	if (read < 0)
	{
		LOG(LIO, LInfo, "SlimeVR Receiver %s: %ls", path.c_str(), hid_error(handle));
		return IMU_STATUS_DISCONNECTED;
	}
	return IMU_STATUS_NORMAL;
}

bool SlimeVRReceiver::parseIMUPacket(SlimeVRTracker &tracker, uint8_t data[16])
{
	uint8_t pktID = data[0];
	uint8_t trkID = data[1];
	switch (pktID)
	{
		case 0:
		{
			LOG(LIO, LTrace, "    Received packet with id %d from tracker %d", pktID, trkID);
			tracker.hasBattery = data[2]&0x80;
			tracker.isPlugged = data[2] == 255;
			tracker.batteryLevel = (data[2]&0x7F)/128.0f;
			tracker.batteryVolts = (data[3]*0.01f)+2.45f;
			tracker.temperature = (data[4] - 128.5f)/2 + 25;
			tracker.signalStrength = data[15];
			uint8_t imuID = data[8];
			tracker.board = (SLIMEVR_BOARD_TYPE)data[5];
			tracker.MCU = (SLIMEVR_MCU_TYPE)data[6];
			if (tracker.board >= SLIMEVR_BOARD_MAX && tracker.board != SLIMEVR_BOARD_DEV_RESERVED)
				tracker.board = SLIMEVR_BOARD_UNKNOWN;
			if (tracker.MCU >= SLIMEVR_MCU_MAX && tracker.MCU != SLIMEVR_MCU_DEV_RESERVED)
				tracker.MCU = SLIMEVR_MCU_UNKNOWN;
			return false;
		}
		case 1:
		{
			LOG(LIO, LTrace, "    Received packet with id %d from tracker %d", pktID, trkID);
			IMUReport report = {};
			constexpr float quatScale = 1.0f / (1<<15), vecScale = 1.0f / (1<<7);
			report.quat = Eigen::Quaternionf(
				*(int16_t*)(data+8) * quatScale,
				*(int16_t*)(data+2) * quatScale,
				*(int16_t*)(data+4) * quatScale,
				*(int16_t*)(data+6) * quatScale);
			report.accel = Eigen::Vector3f(
				*(int16_t*)(data+10) * vecScale,
				*(int16_t*)(data+12) * vecScale,
				*(int16_t*)(data+14) * vecScale);
			report.timestamp = sclock::now(); // TODO: TimeSync
			tracker.reports.push_back(report);
			return true;
		}
		case 2:
		{
			LOG(LIO, LTrace, "    Received packet with id %d from tracker %d", pktID, trkID);
			bool hasBatt = data[2]&0x80;
			tracker.isPlugged = data[2] == 255;
			tracker.batteryLevel = (data[2]&0x7F)/128.0f;
			tracker.batteryVolts = (data[3]*0.01f)+2.45f;
			tracker.temperature = (data[4] - 128.5f)/2 + 25;
			tracker.signalStrength = data[15];
			LOG(LIO, LInfo, "    Temperature is %f", tracker.temperature);

			IMUReport report = {};
			constexpr float scale10 = 1.0f / (1<<10), scale11 = 1.0f / (1<<11), vecScale = 1.0f / (1<<7);
			uint32_t qData = *(uint32_t*)(data+5);
			Eigen::Vector3f quatEnc(
				((qData >> 0 )&0x3FF) * scale10,
				((qData >> 10)&0x7FF) * scale11,
				((qData >> 21)&0x7FF) * scale11
			);
			quatEnc -= Eigen::Vector3f::Constant(0.5f);
			float norm = quatEnc.norm();
			report.quat.w() = std::cos(PI * norm);
			report.quat.vec() = std::sin(PI * norm) * quatEnc/norm;
			report.accel = Eigen::Vector3f(
				*(int16_t*)(data+9) * vecScale,
				*(int16_t*)(data+11) * vecScale,
				*(int16_t*)(data+13) * vecScale);
			report.timestamp = sclock::now(); // TODO: TimeSync
			tracker.reports.push_back(report);
			return true;
		}
		case 3:
			LOG(LIO, LTrace, "    Received packet with id %d from tracker %d", pktID, trkID);
			tracker.status = (SLIMEVR_TRACKER_STATUS)data[2];
			tracker.signalStrength = data[15];
			return false;
		default:
			LOG(LIO, LDarn, "    Received invalid packet with id %d", pktID);
			return false;
	}
}

bool detectSlimeVRReceivers(std::vector<std::shared_ptr<IMUDeviceProvider>> &providers)
{
	bool added = false;
	hid_device_info *devs = hid_enumerate(0x1209, 0x7690);
	for (hid_device_info *dev = devs; dev; dev = dev->next)
	{
		if (std::find_if(providers.begin(), providers.end(), [&dev](auto &p)
			{
				if (p->driver != IMU_DRIVER_SLIMEVR) return false;
				auto sp = static_cast<SlimeVRReceiver*>(p.get());
				return sp->path.compare(dev->path) == 0;
			}) != providers.end())
			continue; // Alraedy added
		LOG(LIO, LInfo, "Adding SlimeVR Receiver %s", dev->path);
		hid_device *handle = hid_open_path(dev->path);
		hid_set_nonblocking(handle, true);
		providers.emplace_back(new SlimeVRReceiver(dev->path, handle));
		added = true;
	}
	hid_free_enumeration(devs);
	return added;
}