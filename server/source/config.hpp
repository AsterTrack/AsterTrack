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

#include "util/eigendef.hpp"
#include "util/blocked_vector.hpp"

#include <vector>
#include <map>
#include <string>
#include <memory> // shared_ptr

#include "blob/parameters.hpp"

/**
 * Parsing and writing of Config and other files
 */


/* Structures */

// Forward-declared opaque structs
struct FrameRecord; // pipeline/frameRecord.hpp
struct TargetTemplate3D; // target/target.hpp
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
	int exposure = 6;
	int gain = 2;
	int filter = 0;
	bool enableStrobe = true;
	int strobeOffset = 0;
	int strobeLength = 10;
	
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

struct GeneralConfig
{
	struct {
		float blobPxStdDev = 0.5f;
		// TODO: WARNING: These simulated targets in config are pointed to!
		// Relevant should we ever want to re-load the config at runtime - make sure to handle all users of this first
		std::vector<TargetTemplate3D> trackingTargets;
		std::vector<CameraCalib> cameraDefinitions;
		std::string cameraDefPath;
	} simulation;
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


/* Functions */

/**
 * Find the file with the highest continuous enumeration for a path format with %d (e.g. "path/to/file_%d.txt")
 * Returns -1 if 0 was not found.
 * For any other number, the next enumerated file (with index+1) does not exist yet.
 */
int findLastFileEnumeration(const char* pathFormat);

void parseGeneralConfigFile(const std::string &path, GeneralConfig &config);
void storeGeneralConfigFile(const std::string &path, const GeneralConfig &config);

void parseCameraConfigFile(const std::string &path, CameraConfigMap &config);
void storeCameraConfigFile(const std::string &path, const CameraConfigMap &config);

void parseCameraCalibrations(const std::string &path, std::vector<CameraCalib> &cameraCalib);
void storeCameraCalibrations(const std::string &path, const std::vector<CameraCalib> &cameraCalib);

void parseTargetCalibrations(const std::string &path, std::vector<TargetTemplate3D> &targetTemplates);
void storeTargetCalibrations(const std::string &path, const std::vector<TargetTemplate3D> &targetTemplates);

unsigned int parseFrameRecords(const std::string &path, std::vector<CameraConfigRecord> &cameras, std::vector<FrameRecord> &frameRecords);
template<typename Iterator>
void dumpFrameRecords(const std::string &path, const Iterator &frameStart, const Iterator &frameEnd, const std::vector<CameraConfigRecord> &cameras);

void parseTrackingResults(std::string path, std::vector<FrameRecord> &frameRecords, unsigned int frameOffset);
template<typename Iterator>
void dumpTrackingResults(std::string path, const Iterator &frameStart, const Iterator &frameEnd, unsigned int frameOffset);

SequenceData parseSequenceDatabase(const std::string &path, std::vector<CameraID> &cameraIDs);
void dumpSequenceDatabase(const std::string &path, const SequenceData &sequences, const std::vector<CameraID> &cameraIDs);

std::vector<std::shared_ptr<TargetView>> parseTargetViewRecords(const std::string &path,
	const BlockedQueue<std::shared_ptr<FrameRecord>> &frameRecords);
void dumpTargetViewRecords(const std::string &path, const std::vector<std::shared_ptr<TargetView>> &views);

bool parseTargetAssemblyStage(const std::string &path, TargetAssemblyBase &base);
void dumpTargetAssemblyStage(const std::string &path, const TargetAssemblyBase &base);

bool parseTargetObjFile(const std::string &path, std::vector<TargetTemplate3D> &targets, float fov, float size);
void writeTargetObjFile(const std::string &path, const TargetTemplate3D &targetTemplate);

#endif // CONFIG_H