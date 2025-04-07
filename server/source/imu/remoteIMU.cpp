
#include "imu/device.hpp"

#include "util/log.hpp"

//#include <unordered_map>

class RemoteIMU : public IMUDevice
{
public:
	RemoteIMU(int device, bool hasMag = false, bool isFused = false)
		: IMUDevice(hasMag, isFused, IMU_DRIVER_REMOTE, 0, device), sampleCount(0)
	{}

	~RemoteIMU() = default;

	std::size_t sampleCount;
};

class RemoteIMUSingleton;
static std::shared_ptr<RemoteIMUSingleton> remoteIMUSingleton;

class RemoteIMUSingleton : public IMUDeviceProvider
{
public:

	std::size_t deviceCount;

	RemoteIMUSingleton() : IMUDeviceProvider(IMU_DRIVER_REMOTE), deviceCount(0)
	{
		latencyDescriptions.descriptions.push_back("Remote RX");
		latencyDescriptions.descriptions.push_back("Fusion");
	}

	~RemoteIMUSingleton()
	{
		remoteIMUSingleton = nullptr;
	}

	IMUDeviceProviderStatus poll(int &updatedDevices, int &changedDevices);
};

IMUDeviceProviderStatus RemoteIMUSingleton::poll(int &updatedDevices, int &changedDevices)
{
	if (deviceCount < devices.size())
	{ // Detected added RemoteIMU
		changedDevices = devices.size() - deviceCount;
		deviceCount = devices.size();
	}
	for (auto &remoteIMU : devices)
	{ // Check for updated RemoteIMUs
		auto &imu = *(RemoteIMU*)remoteIMU.get();
		std::size_t count = imu.isFused? imu.samplesFused.getView().size() : imu.samplesRaw.getView().size();
		if (imu.sampleCount < count)
		{ // Detected updated RemoteIMU
			updatedDevices++;
			imu.sampleCount = count;
		}
	}
	return IMU_STATUS_NORMAL;
}

bool initialiseRemoteIMUs(std::vector<std::shared_ptr<IMUDeviceProvider>> &providers)
{
	auto singletonIt = std::find_if(providers.begin(), providers.end(),
		[](auto &p){ return p->driver == IMU_DRIVER_REMOTE; });
	if (singletonIt != providers.end())
	{
		if (!remoteIMUSingleton)
			remoteIMUSingleton = std::static_pointer_cast<RemoteIMUSingleton>(*singletonIt);
		return false;
	}
	remoteIMUSingleton = std::make_shared<RemoteIMUSingleton>();
	providers.push_back(remoteIMUSingleton);
	return true;
}

std::shared_ptr<IMUDevice> registerRemoteIMU(int deviceID)
{
	auto remote = remoteIMUSingleton;
	if (!remote) return nullptr;
	auto remoteIMUIt = std::find_if(remote->devices.begin(), remote->devices.end(),
		[&](const auto &imu) { return imu->device == deviceID; });
	if (remoteIMUIt != remote->devices.end())
		return std::static_pointer_cast<IMUDevice>(*remoteIMUIt);
	auto remoteIMU = std::make_shared<RemoteIMU>(deviceID);
	remote->devices.push_back(remoteIMU);
	return remoteIMU;
}

IMUDeviceProvider *getRemoteIMUProvider()
{
	return remoteIMUSingleton? remoteIMUSingleton.get() : nullptr;
}