
#include "imu/device.hpp"

#include "util/log.hpp"

//#include <unordered_map>

class RemoteIMU : public IMUDevice
{
public:
	RemoteIMU(std::string path, bool hasMag = false, bool isFused = false)
		: IMUDevice(IMUIdent{ IMU_DRIVER_REMOTE, path }, hasMag, isFused), sampleCount(0)
	{
		timingRecord.supportsLatency = true;
		timingRecord.latencyDescriptions.push_back("Remote RX");
		timingRecord.latencyDescriptions.push_back("Fusion");
	}

	~RemoteIMU() = default;

	std::string getDescriptor();

	std::size_t sampleCount;
};

class RemoteIMUSingleton;
static std::shared_ptr<RemoteIMUSingleton> remoteIMUSingleton;

class RemoteIMUSingleton : public IMUDeviceProvider
{
public:

	IMUDeviceList removedIMUs;
	IMUDeviceList addedIMUs;

	RemoteIMUSingleton() : IMUDeviceProvider(IMU_DRIVER_REMOTE)
	{
		timingRecord.supportsLatency = true;
		timingRecord.latencyDescriptions.push_back("Remote RX");
		timingRecord.latencyDescriptions.push_back("Fusion");
	}

	~RemoteIMUSingleton()
	{
		remoteIMUSingleton = nullptr;
	}

	IMUDeviceProviderStatus poll(int &updated, IMUDeviceList &removed, IMUDeviceList &added);
	std::string getDescriptor();
};

IMUDeviceProviderStatus RemoteIMUSingleton::poll(int &updated, IMUDeviceList &removed, IMUDeviceList &added)
{
	for (auto &remoteIMU : devices)
	{ // Check for updated RemoteIMUs
		if (!remoteIMU) continue;
		auto &imu = *(RemoteIMU*)remoteIMU.get();
		std::size_t count = imu.isFused? imu.samplesFused.getView().size() : imu.samplesRaw.getView().size();
		if (imu.sampleCount < count)
		{ // Detected updated RemoteIMU
			updated++;
			imu.sampleCount = count;
		}
	}
	std::move(removedIMUs.begin(), removedIMUs.end(), std::back_inserter(removed));
	std::move(addedIMUs.begin(), addedIMUs.end(), std::back_inserter(added));
	removedIMUs.clear();
	addedIMUs.clear();
	return devices.empty()? IMU_STATUS_NO_DEVICES : IMU_STATUS_DEVICES_CONNECTED;
}

std::string RemoteIMUSingleton::getDescriptor()
{
	if (devices.empty()) return "";
	return asprintf_s("%d Remote IMUs", (int)devices.size());
}

std::string RemoteIMU::getDescriptor()
{
	return asprintf_s("Remote IMU %s", id.string.c_str());
}

bool initialiseRemoteIMUs(IMUDeviceProviderList &providers)
{
	auto singletonIt = std::find_if(providers.begin(), providers.end(),
		[](auto &p){ return p->driver == IMU_DRIVER_REMOTE; });
	if (singletonIt != providers.end())
	{
		if (!remoteIMUSingleton)
			remoteIMUSingleton = std::static_pointer_cast<RemoteIMUSingleton>(*singletonIt);
		return false;
	}
	if (!remoteIMUSingleton)
		remoteIMUSingleton = std::make_shared<RemoteIMUSingleton>();
	providers.push_back(remoteIMUSingleton);
	return true;
}

std::shared_ptr<IMUDevice> registerRemoteIMU(std::string path)
{
	auto remote = remoteIMUSingleton;
	if (!remote) return nullptr;
	auto remoteIMUIt = std::find_if(remote->devices.begin(), remote->devices.end(),
		[&](const auto &imu) { return imu && imu->id.path().compare(path) == 0; });
	if (remoteIMUIt != remote->devices.end())
		return *remoteIMUIt;
	auto imu = std::make_shared<RemoteIMU>(path);
	remote->devices.push_back(imu);
	remote->addedIMUs.push_back(imu);
	return imu;
}

void removeRemoteIMU(std::shared_ptr<IMUDevice> remoteIMU)
{
	auto remote = remoteIMUSingleton;
	if (!remote) return;
	auto remoteIMUIt = std::find_if(remote->devices.begin(), remote->devices.end(),
		[&](const auto &imu) { return imu == remoteIMU; });
	if (remoteIMUIt != remote->devices.end())
		*remoteIMUIt = nullptr;
	remote->removedIMUs.push_back(std::move(remoteIMU));
}

IMUDeviceProvider *getRemoteIMUProvider()
{
	return remoteIMUSingleton? remoteIMUSingleton.get() : nullptr;
}