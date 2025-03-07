
#include "imu/imu.hpp"

#include "util/log.hpp"

#include "hidapi/hidapi.h"


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

	SlimeVRTracker(int provider, int device) : IMUDevice(IMU_DRIVER_SLIMEVR, provider, device)
	{}
};

#define TRACKER_REPORT_SIZE 16 // 1B Header + 12B Data + 2B Timestamp (or 1B Header + 14B Data)
#define HID_HEADER_SIZE 0 // 1B Header + 12B Data + 2B Timestamp (or 1B Header + 14B Data)
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
	bool parseDataPacket(SlimeVRTracker &tracker, uint8_t data[TRACKER_REPORT_SIZE], TimePoint_t &timestamp);
};

enum PACKET_HEADER_TYPE
{
	TYPE_INFO = 0,
	TYPE_IMU = 1,
	TYPE_IMU_STATUS = 2,
	TYPE_INVALID = 3,
	HEADER_SKIP = 0b11111111,
};

IMUDeviceProviderStatus SlimeVRReceiver::poll(int &updatedDevices, int &changedDevices)
{
	thread_local uint8_t data[64+8]; 
	int reports = 0;
	int read;
	LOG(LIO, LDebug, "Polling SlimeVR Receiver %s!", path.c_str());
	while ((read = hid_read(handle, data, sizeof(data))) > 0)
	{
		TimePoint_t recvTime = sclock::now();
		uint8_t *header = data;
		uint8_t size = read;
		if (read > 64)
		{ // Multiple reports possible
			int reportID = data[0];
			LOG(LIO, LDebug, "  SlimeVR Receiver Report %d sent %d bytes!", reportID, read);
			header = data+1;
		}

		uint8_t *packet = header+HID_HEADER_SIZE;
		int packets = (size-HID_HEADER_SIZE)/TRACKER_REPORT_SIZE;
		LOG(LIO, LDebug, "  SlimeVR Receiver sent %d bytes (%d packets)!", size, packets);
		for (int p = 0; p < packets; p++)
		{
			int i = p*TRACKER_REPORT_SIZE;
			if (packet[i+0] == HEADER_SKIP) continue;
			uint8_t tracker_id = packet[i+1];
			LOG(LIO, LTrace, "    Packet %d from tracker %d!", packet[i+0], packet[i+1]);
			if (devices.size() <= tracker_id)
				devices.resize(tracker_id+1);
			if (!devices[tracker_id])
			{
				// TODO: Use receiver and tracker serial numbers
				devices[tracker_id] = std::make_shared<SlimeVRTracker>(-1, tracker_id);
				LOG(LIO, LInfo, "    Registered new tracker %d!", tracker_id);
				changedDevices++;
			}
			SlimeVRTracker &tracker = *static_cast<SlimeVRTracker*>(devices[tracker_id].get());
			TimePoint_t sampleTime = recvTime;
			if (parseDataPacket(tracker, packet+i, sampleTime))
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

bool SlimeVRReceiver::parseDataPacket(SlimeVRTracker &tracker, uint8_t data[TRACKER_REPORT_SIZE], TimePoint_t &timestamp)
{
	uint8_t type = data[0];
	uint8_t tracker_id = data[1];
	switch (type)
	{
		case TYPE_INFO:
		{
			LOG(LIO, LTrace, "    Received packet with id %d from tracker %d", type, tracker_id);
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
		case TYPE_IMU:
		{
			LOG(LIO, LTrace, "    Received packet with id %d from tracker %d", type, tracker_id);
			IMUSample report = {};
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
			report.timestamp = timestamp;
			tracker.samples.push_back(report);
			return true;
		}
		case TYPE_IMU_STATUS:
		{
			LOG(LIO, LTrace, "    Received packet with id %d from tracker %d", type, tracker_id);
			bool hasBatt = data[2]&0x80;
			tracker.isPlugged = data[2] == 255;
			tracker.batteryLevel = (data[2]&0x7F)/128.0f;
			tracker.batteryVolts = (data[3]*0.01f)+2.45f;
			tracker.temperature = (data[4] - 128.5f)/2 + 25;
			tracker.signalStrength = data[15];
			LOG(LIO, LInfo, "    Temperature is %f", tracker.temperature);

			IMUSample report = {};
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
			report.timestamp = timestamp;
			LOG(LIO, LTrace, "    Received IMU sample from tracker %d with latency of %.3fms",
				tracker_id, dtMS(report.timestamp, sclock::now()));
			tracker.samples.push_back(report);
			return true;
		}
		case TYPE_INVALID:
			LOG(LIO, LTrace, "    Received packet with id %d from tracker %d", type, tracker_id);
			tracker.status = (SLIMEVR_TRACKER_STATUS)data[2];
			tracker.signalStrength = data[15];
			return false;
		default:
			LOG(LIO, LDarn, "    Received invalid packet with id %d", type);
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
		hid_device *handle = hid_open_path(dev->path);
		if (!handle)
		{
			LOG(LIO, LWarn, "Failed to add original SlimeVR Receiver %s", dev->path);
			continue;
		}
		hid_set_nonblocking(handle, true);
		providers.emplace_back(new SlimeVRReceiver(dev->path, handle));
		LOG(LIO, LInfo, "Adding SlimeVR Receiver %s", dev->path);
		added = true;
	}
	hid_free_enumeration(devs);
	return added;
}