/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef CONFIG_H
#define CONFIG_H

#include "pipeline/record.hpp"
#include "blob/parameters.hpp"
#include "target/parameters.hpp"
#include "target/target.hpp"
#include "imu/imu.hpp"
#include "tracking/virtual.hpp"

#include "util/eigendef.hpp"
#include "util/blocked_vector.hpp"

#include <vector>
#include <map>
#include <string>
#include <memory> // shared_ptr

/**
 * Parsing and writing of Config and other files
 */


/* Structures */

// Forward-declared opaque structs
struct FrameRecord; // pipeline/record.hpp
struct TargetCalibration3D; // target/target.hpp
struct TargetView; // calib_target/assembly.hpp
struct TargetAssemblyBase; // calib_target/assembly.hpp
struct SequenceData; // point/sequence_data.hpp

struct CameraConfig
{
	std::string label;
	int width = 1280;
	int height = 800;
	bool synchronised = true;
	int framerate = 144; // Only used if synchronised == false
	int exposure = 20;
	int gain = 2;
	int filter = 0;
	bool enableStrobe = true;
	int strobeOffset = 0;
	int strobeLength = 55;
	
	BlobProcessingParameters blobProcessing = {};
	bool shareBlobProcessing = true;
};

struct ControllerConfig
{
	int framerate = 120;
	//SyncSource syncSource = SYNC_INTERNAL;
	// TODO: Should contain relations between controllers
	// E.g. not just whether it takes input, but from where
	// So syncSource itself is not very helpful alone
	// Should be configurable in UI in "Devices" window
};

struct TrackerConfig
{
	enum TrackerType
	{ // Saved on disk, don't change
		TRACKER_TARGET = 1,
		TRACKER_MARKER = 2,
		TRACKER_VIRTUAL = 3
	};

	enum TrackerTrigger
	{ // Saved on disk, don't change
		TRIGGER_BY_DEFAULT = 0,
		TRIGGER_ONLY_MANUALLY = (1 << 0),
		TRIGGER_ON_IMU_CONNECT = (1 << 1),
		TRIGGER_ON_IO_CONNECT = (1 << 2),
		TRIGGER_MAX,
		TRIGGER_MASK = ((TRIGGER_MAX-1)*2-1),
		TRIGGER_FLAG_MASK = (TRIGGER_MASK&~1)
	};

	enum TrackerExpose
	{ // Saved on disk, don't change
		EXPOSE_BY_DEFAULT = 0,
		EXPOSE_ONCE_TRIGGERED = 1,
		EXPOSE_ONCE_TRACKED = 2,
		EXPOSE_MAX
	};

	enum TrackerRole
	{ // Saved on disk, don't change
		ROLE_TRACKER = 0,
		ROLE_HMD = 1,
		ROLE_CONTROLLER = 2,
		ROLE_MAX
	};

	// General Config
	int id;
	std::string label;
	TrackerType type;
	bool isSimulated = false;
	TrackerTrigger trigger = TRIGGER_BY_DEFAULT;
	TrackerExpose expose = EXPOSE_BY_DEFAULT;
	TrackerRole role = ROLE_TRACKER;

	// Tracker specific dirty flags
	bool configDirty = false;
	bool targetDirty = false;

	// Target-specific Config
	TargetCalibration3D calib;
	TargetDetectionConfig detectionConfig;

	// Marker-specific Config
	float markerSize;

	// Virtual-specific Config
	TrackerVirtualConfig virtConfig;

	// Optional IMU
	IMUIdent imuIdent = {};
	IMUCalib imuCalib;

	// Current state
	std::shared_ptr<IMU> imu;
	bool triggered = false, tracked = false;
	bool exposed = false;
	int connected = 0;

	TrackerConfig(int ID, std::string label, TrackerType type)
		: id(ID), label(label), type(type) {}

	TrackerConfig(int ID, std::string label, TargetCalibration3D target, TargetDetectionConfig detectionConfig, bool isSimulated = false)
		: id(ID), label(label), type(TRACKER_TARGET), calib(std::move(target)), detectionConfig(detectionConfig),
		  isSimulated(isSimulated), configDirty(!isSimulated), targetDirty(!isSimulated) {}

	TrackerConfig(int ID, std::string label, float markerSize)
		: id(ID), label(label), type(TRACKER_MARKER), markerSize(markerSize) {}
};

struct GeneralConfig
{
	struct {
		float blobPxStdDev = 0.5f;
		std::map<std::string, TargetCalibration3D> trackingTargets;
		std::vector<CameraCalib> cameraDefinitions;
		std::string cameraDefPath;
	} simulation = {};

	struct {
		bool vrpn_auto_enable;
		std::string vrpn_host;
		int vrpn_port;

		bool vmc_auto_enable;
		std::string vmc_host;
		int vmc_port;
	} integrations = {};
};

struct CameraConfigMap
{
	std::vector<CameraConfig> configurations;
	std::map<CameraID, int> cameraConfigs;

	int getCameraConfigIndex(CameraID id)
	{
		if (configurations.empty())
			configurations.push_back({});
		return cameraConfigs.insert(std::make_pair(id, 0)).first->second;
	}

	CameraConfig &getCameraConfig(CameraID id)
	{
		std::size_t index = getCameraConfigIndex(id);
		//assert(configurations.size() > index);
		return configurations[index];
	}

	const CameraConfig *getCameraConfig(CameraID id) const
	{
		auto it = cameraConfigs.find(id);
		if (it == cameraConfigs.end()) return nullptr;
		//assert(configurations.size() > it->second);
		return &configurations[it->second];
	}
};

// Basic record of camera config needed for replay
struct CameraConfigRecord
{
	CameraID ID;
	int frameX, frameY;
};


/* Variables */

extern const std::string persistentConfigFolder, trackerConfigFolder, recordingsFolder, permanentStoreFolder, temporaryStoreFolder;

/* Functions */

/**
 * Find the highest continuous enumeration for a file matching the full path format with %d (e.g. "path/to/file_%d.txt")
 * The next enumerated file (with index+1) does not exist yet.
 * There may be enumerated files matching the pattern that have an index greater than index+1.
 */
int findLastFileEnumeration(const std::string &pathFormat);

/**
 * Find the highest enumeration for a file matching the partial path format with %d (e.g. "path/to/file_%d")
 * The next enumerated file (with index+1) does not exist yet.
 * All enumerated files matching the pattern have an index lower than index+1.
 */
int findHighestFileEnumeration(const char* path, const char* nameFormat, const char *extension, bool allowTrailingString = true);

std::optional<ErrorMessage> touchFile(const std::string &path);

HANDLE_ERROR parseGeneralConfigFile(const std::string &path, GeneralConfig &config);
HANDLE_ERROR storeGeneralConfigFile(const std::string &path, const GeneralConfig &config);

HANDLE_ERROR parseCameraConfigFile(const std::string &path, CameraConfigMap &config);
HANDLE_ERROR storeCameraConfigFile(const std::string &path, const CameraConfigMap &config);

HANDLE_ERROR parseLensPresets(const std::string &path, std::map<int, LensCalib> &lensCalib, int &defaultLens);
HANDLE_ERROR storeLensPresets(const std::string &path, const std::map<int, LensCalib> &lensCalib, int defaultLens);

HANDLE_ERROR parseCameraCalibrations(const std::string &path, std::vector<CameraCalib> &cameraCalib);
HANDLE_ERROR storeCameraCalibrations(const std::string &path, const std::vector<CameraCalib> &cameraCalib);

HANDLE_ERROR parseTrackerConfiguration(const std::string &folder, int id, std::optional<TrackerConfig> &trackerConfig);
HANDLE_ERROR storeTrackerConfiguration(const std::string &folder, TrackerConfig &tracker);

HANDLE_ERROR parseTrackerConfigurations(const std::string &folder, std::vector<TrackerConfig> &trackerConfig);
HANDLE_ERROR storeTrackerConfigurations(const std::string &folder, std::vector<TrackerConfig> &trackerConfig);

HANDLE_ERROR parseTrackerConfigurationsLegacy(const std::string &path, std::vector<TrackerConfig> &trackerConfig);

HANDLE_ERROR parseRecording(const std::string &path, std::vector<CameraConfigRecord> &cameras, TrackingRecord &record, std::size_t &frameOffset, bool separate);
HANDLE_ERROR saveRecording(const std::string &path, const std::vector<CameraConfigRecord> &cameras, const TrackingRecord &record, std::size_t begin, std::size_t end);

HANDLE_ERROR parseTrackingResults(std::string &path, TrackingRecord &record, std::size_t frameOffset);
HANDLE_ERROR saveTrackingResults(std::string &path, const TrackingRecord &record, std::size_t begin, std::size_t end, std::size_t frameOffset);

HANDLE_ERROR parseSequenceDatabase(const std::string &path, std::vector<CameraID> &cameraIDs, SequenceData &sequences);
HANDLE_ERROR dumpSequenceDatabase(const std::string &path, const SequenceData &sequences, const std::vector<CameraID> &cameraIDs);

HANDLE_ERROR parseTargetViewRecords(const std::string &path,
	const BlockedQueue<std::shared_ptr<FrameRecord>> &frameRecords, std::vector<std::shared_ptr<TargetView>> &views);
HANDLE_ERROR dumpTargetViewRecords(const std::string &path, const std::vector<std::shared_ptr<TargetView>> &views);

HANDLE_ERROR parseTargetAssemblyStage(const std::string &path, TargetAssemblyBase &base);
HANDLE_ERROR dumpTargetAssemblyStage(const std::string &path, const TargetAssemblyBase &base);

HANDLE_ERROR parseTargetObjFile(const std::string &path, std::map<std::string, TargetCalibration3D> &targets, float fov, float size, bool filterGroups = true);
HANDLE_ERROR writeTargetObjFile(const std::string &path, const std::string &label, const TargetCalibration3D &target);

#endif // CONFIG_H