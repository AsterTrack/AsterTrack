
#include "imu/device.hpp"

#include "util/log.hpp"

#include "hidapi/hidapi.h"

/**
 * Modified SlimeVR firmware that mainly changes the protocols to include timesync
 * TimeSync is required to get exact timesteps of the IMU samples to fuse with optical data correctly
 * This adds ~2-3 bytes overhead to communication, but protocol saves some bytes elsewhere
 */

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

class AsterTrackTracker : public IMUDevice
{
public:
	SLIMEVR_BOARD_TYPE board;
	SLIMEVR_MCU_TYPE MCU;
	SLIMEVR_TRACKER_STATUS status;

	AsterTrackTracker(int provider, int device)
		: IMUDevice(false, true, IMU_DRIVER_ASTERTRACK, provider, device)
	{}

	~AsterTrackTracker() = default;
};

#define TRACKER_INFO_SIZE 11 // 1B Header + 10B Info
#define TRACKER_STATUS_SIZE 5 // 1B Header + 3B Status + !B RSSI
#define TRACKER_REPORT_SIZE 15 // 1B Header + 12B Data + 2B Timestamp (or 1B Header + 14B Data)
#define HID_HEADER_SIZE 4
class AsterTrackReceiver : public IMUDeviceProvider
{
public:
	std::string path;
	hid_device *handle;
	TimeSync timeSync;

	AsterTrackReceiver(const char *path, hid_device *handle)
		: path(path), handle(handle),
		IMUDeviceProvider(IMU_DRIVER_ASTERTRACK)
	{
		ResetTimeSync(timeSync);
		// Unknown latencies of fusion and radio transmission, only the sample timestamp is sent
		latencyDescriptions.descriptions.push_back("USB TX");
		latencyDescriptions.descriptions.push_back("USB RX");
		latencyDescriptions.descriptions.push_back("Fusion");
	}

	~AsterTrackReceiver()
	{
		hid_close(handle);
	}

	IMUDeviceProviderStatus poll(int &updatedDevices, int &changedDevices);
	bool parseInfoPacket(AsterTrackTracker &tracker, uint8_t data[TRACKER_INFO_SIZE]);
	bool parseStatusPacket(AsterTrackTracker &tracker, uint8_t data[TRACKER_STATUS_SIZE]);
	bool parseDataPacket(AsterTrackTracker &tracker, uint8_t data[TRACKER_REPORT_SIZE], TimePoint_t &timestamp);
};

enum PACKET_HEADER_TYPE
{
	HEADER_PAIR = 0b00000000, // blocks type 0b000 as tracker_id 0 does exist
	TYPE_INFO_STATUS = 0b001,
	TYPE_IMU_CAYLEY = 0b010,
	// Remaining types are for future uses / revisions
	TYPE_GENERIC_HID = 0b100, // First byte is used to specify exact HID format (tbd)
	TYPE_REGISTER = 0b111,
	HEADER_SKIP = 0b11111111,
};

IMUDeviceProviderStatus AsterTrackReceiver::poll(int &updatedDevices, int &changedDevices)
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

		uint16_t timestampUS = ((uint16_t*)header)[0];
		LOG(LIO, LTrace, "Raw receiver timestamp of last packet is %d", timestampUS);
		auto packetSentSync = UpdateTimeSync(timeSync, timestampUS, 1<<16, recvTime);
		TimePoint_t sentTime = packetSentSync.second;
		if (recordTimeSyncMeasurements)
			timeSyncMeasurements.push_back({ packetSentSync.first, sentTime, recvTime });

		uint8_t *packet = header+HID_HEADER_SIZE;
		int packets = (size-HID_HEADER_SIZE)/TRACKER_REPORT_SIZE;
		LOG(LIO, LDebug, "  SlimeVR Receiver sent %d bytes (%d packets)!", size, packets);
		for (int p = 0; p < packets; p++)
		{
			int i = p*TRACKER_REPORT_SIZE;
			if (packet[i+0] == HEADER_SKIP) continue;
			uint8_t tracker_id = packet[i+0] & 0b11111;
			LOG(LIO, LTrace, "    Packet %d from tracker %d!", packet[i+0] >> 5, tracker_id);
			assert(tracker_id < 31);
			if (devices.size() <= tracker_id)
				devices.resize(tracker_id+1);
			if (!devices[tracker_id])
			{
				// TODO: Use receiver and tracker serial numbers
				devices[tracker_id] = std::make_shared<AsterTrackTracker>(-1, tracker_id);
				devices[tracker_id]->hasMag = false;
				LOG(LIO, LInfo, "    Registered new tracker %d!", tracker_id);
				changedDevices++;
			}
			AsterTrackTracker &tracker = *static_cast<AsterTrackTracker*>(devices[tracker_id].get());
			TimePoint_t sampleTime = recvTime;
			if (parseDataPacket(tracker, packet+i, sampleTime))
			{
				if (recordLatencyMeasurements)
				{ // TODO: Move latency measurement up after fusion
					// E.g. store LatencyMeasurement in IMUReport until after fusion to add fusion latency ontop
					latencyMeasurements.push_back({ sampleTime, { 
						(uint16_t)dtUS(sampleTime, sentTime),
						(uint16_t)dtUS(sampleTime, recvTime) } } );
				}
				updatedDevices++;
			}
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

bool AsterTrackReceiver::parseInfoPacket(AsterTrackTracker &tracker, uint8_t data[TRACKER_INFO_SIZE])
{
	uint8_t header = data[0];
	tracker.board = (SLIMEVR_BOARD_TYPE)data[1];
	tracker.MCU = (SLIMEVR_MCU_TYPE)data[2];
	if (tracker.board >= SLIMEVR_BOARD_MAX && tracker.board != SLIMEVR_BOARD_DEV_RESERVED)
		tracker.board = SLIMEVR_BOARD_UNKNOWN;
	if (tracker.MCU >= SLIMEVR_MCU_MAX && tracker.MCU != SLIMEVR_MCU_DEV_RESERVED)
		tracker.MCU = SLIMEVR_MCU_UNKNOWN;
	// TODO: Parse remaining info packet
	uint8_t imuID = data[4];
	uint8_t magID = data[5];
	uint16_t fw_date = *(uint16_t*)&data[6];
	uint8_t fw_major = data[8];
	uint8_t fw_minor = data[9];
	uint8_t fw_patch = data[10];
	return true;
}

bool AsterTrackReceiver::parseStatusPacket(AsterTrackTracker &tracker, uint8_t data[TRACKER_STATUS_SIZE])
{
	uint8_t header = data[0];
	tracker.hasBattery = data[1]&0x80;
	tracker.isPlugged = data[1] == 255;
	tracker.batteryLevel = (data[1]&0x7F)/128.0f;
	tracker.batteryVolts = (data[2]*0.01f)+2.45f;
	tracker.temperature = (data[3] - 128.5f)/2 + 25;
	tracker.signalStrength = data[4];
	return true;
}

bool AsterTrackReceiver::parseDataPacket(AsterTrackTracker &tracker, uint8_t data[TRACKER_REPORT_SIZE], TimePoint_t &timestamp)
{
	uint8_t header = data[0];
	uint8_t type = header >> 5;
	uint8_t tracker_id = header & 0b11111;
	switch (type)
	{
		case TYPE_IMU_CAYLEY:
		{
			IMUSampleFused report = {};
			constexpr float quatScale = 1.0f / (1<<15), vecScale = 1.0f / (1<<7);
			Eigen::Vector3f quatEnc(
				*(int16_t*)(data+1) * quatScale,
				*(int16_t*)(data+3) * quatScale,
				*(int16_t*)(data+5) * quatScale);
			// Inverse Exponential Map
			/* float norm = quatEnc.norm();
			report.quat.w() = std::cos(PI * norm);
			report.quat.vec() = std::sin(PI * norm) * quatEnc/norm; */
			// Inverse Cayley Transform
			float norm = 2 / (1 + quatEnc.squaredNorm());
			report.quat.vec() = quatEnc * norm;
			report.quat.w() = norm-1;
			// Acceleration
			report.accel = Eigen::Vector3f(
				*(int16_t*)(data+7) * vecScale,
				*(int16_t*)(data+9) * vecScale,
				*(int16_t*)(data+11) * vecScale);
			// Realign spaces
			Eigen::Matrix3f transform = Eigen::DiagonalMatrix<float, 3>(1, -1, 1);
			report.quat = Eigen::Quaternionf(transform * report.quat.toRotationMatrix() * transform);
			report.accel = transform  * report.accel;
			// Timestamp and TimeSync
			uint16_t timestamp_us = *(uint16_t*)(data+13);
			report.timestamp = GetTimeSynced(timeSync, timestamp_us, 1 << 16);
			timestamp = report.timestamp;
			LOG(LIO, LTrace, "    Received IMU sample from tracker %d with latency of %.3fms",
				tracker_id, dtMS(report.timestamp, sclock::now()));
			tracker.samplesFused.push_back(report);
			return true;
		}
		case TYPE_REGISTER:
			// IMU that is paired but not connected - don't care for it right now, already registered anyway
			return false;
		default:
			LOG(LIO, LDarn, "    Received invalid packet with id %d", type);
			return false;
	}
}

bool detectAsterTrackReceivers(std::vector<std::shared_ptr<IMUDeviceProvider>> &providers)
{
	bool added = false;
	hid_device_info *devs = hid_enumerate(0x1209, 0x7691);
	for (hid_device_info *dev = devs; dev; dev = dev->next)
	{
		if (std::find_if(providers.begin(), providers.end(), [&dev](auto &p)
			{
				if (p->driver != IMU_DRIVER_ASTERTRACK) return false;
				auto sp = static_cast<AsterTrackReceiver*>(p.get());
				return sp->path.compare(dev->path) == 0;
			}) != providers.end())
			continue; // Alraedy added
		hid_device *handle = hid_open_path(dev->path);
		if (!handle)
		{
			LOG(LIO, LWarn, "Failed to add custom SlimeVR Receiver %s", dev->path);
			continue;
		}
		hid_set_nonblocking(handle, true);
		providers.push_back(std::make_shared<AsterTrackReceiver>(dev->path, handle));
		LOG(LIO, LInfo, "Adding SlimeVR Receiver %s", dev->path);
		added = true;
	}
	hid_free_enumeration(devs);
	return added;
}