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

#include "config.hpp"

#include "pipeline/record.hpp"
#include "target/target.hpp"
#include "point/sequence_data.hpp"
#include "calib_target/assembly.hpp"
#include "imu/device.hpp"

#include "util/eigenutil.hpp"
#include "util/blocked_vector.hpp"
#include "util/log.hpp"

#ifndef NDEBUG
#define JSON_NOEXCEPTION
#define JSON_PARSE_TRY_BLOCK
#define JSON_PARSE_CATCH_BLOCK
#else
#define JSON_PARSE_TRY_BLOCK try
#define JSON_PARSE_CATCH_BLOCK \
	catch(json::exception e) \
	{ \
		LOG(LDefault, LWarn, "Failed to fully parse JSON file '%s': '%s'", path.c_str(), e.what()); \
		return asprintf_s("Failed to parse '%s'!", path.c_str()); \
	}

#endif

#include "nlohmann/json.hpp"
using json = nlohmann::json;

#include <set>
#include <fstream>
#include <iomanip>
#include <filesystem>

const std::string persistentConfigFolder = "config";
const std::string trackerConfigFolder = "store/trackers";
const std::string recordingsFolder = "recordings";
const std::string permanentStoreFolder = "store";
const std::string temporaryStoreFolder = "dump";

/**
 * Parsing and writing of Config and other files
 */

std::optional<ErrorMessage> touchFile(const std::string &path)
{
	// Create directories
	std::filesystem::create_directories(std::filesystem::path(path).remove_filename());
	// Write file
	std::ofstream fs(path);
	if (!fs.is_open()) return asprintf_s("Failed to open '%s' for writing!", path.c_str());
	fs.close();
	if (fs.fail()) return asprintf_s("Failed to close '%s' after writing!", path.c_str());
	return std::nullopt;
}

std::optional<ErrorMessage> writeJSON(const std::string &path, const json &data)
{
	// Serialise JSON
	std::stringstream ss;
	ss << std::setfill('\t') << std::setw(1) << data;
	// Create directories
	std::filesystem::create_directories(std::filesystem::path(path).remove_filename());
	// Write file
	std::ofstream fs(path);
	if (!fs.is_open()) return asprintf_s("Failed to open '%s' for writing!", path.c_str());
	fs << ss.rdbuf();
	if (fs.fail()) return asprintf_s("Failed to write '%s'!", path.c_str());
	fs.close();
	if (fs.fail()) return asprintf_s("Failed to close '%s' after writing!", path.c_str());
	// TODO: Verify file was written
	return std::nullopt;
}

std::optional<ErrorMessage> readJSON(const std::string &path, json &data)
{
	// Read file
	std::ifstream fs(path);
	if (!fs.is_open()) return ErrorMessage(asprintf_s("Failed to open '%s' for reading!", path.c_str()), ENOENT);
	fs >> data;
	if (fs.fail()) return asprintf_s("Failed to read '%s'!", path.c_str());
	fs.close();
	if (fs.fail()) return asprintf_s("Failed to close '%s' after reading!", path.c_str());
	return std::nullopt;
}

CameraID parseCameraID(json &jsID)
{
	// Previously written and interpreted as a int32_t, now as uint32_t. Convert negative sign to larger uint32_t.
	int64_t parseID = jsID.get<int64_t>();
	CameraID ID = std::abs(parseID);
	if (parseID < 0) ID |= 0x80000000;
	return ID;
}

int findHighestFileEnumeration(const char* path, const char* nameFormat, const char *extension, bool allowTrailingString)
{
	if (!std::filesystem::exists(std::filesystem::path(path))) return -1;
	int maxNum = -1;
	// Test both for exact match of filename and only beginning
	// Need %n at the end to check for full match even past %d
	// And thus need two checks since %*s apparently doesn't match a zero-length string
	std::string format_exact = std::string(nameFormat) + "%n";
	std::string format_partial = std::string(nameFormat) + "%*s%n";
	for (const auto &file : std::filesystem::directory_iterator(path))
	{
		if (file.path().extension().compare(extension) != 0) continue;
		const std::string &str = file.path().stem().string();
		int read, num, end;
		read = std::sscanf(str.c_str(), format_exact.c_str(), &num, &end);
		if (read != 1 || end != str.size())
		{ // Check if there's just a string after the nameFormat
			if (allowTrailingString)
				read = std::sscanf(str.c_str(), format_partial.c_str(), &num, &end);
			if (read != 1 || end != str.size())
				continue;
		}
		maxNum = std::max(maxNum, num);
	}
	return maxNum;
}

int findLastFileEnumeration(const std::string &pathFormat)
{
	int i = -1;
	std::string obsPath;
	do obsPath = asprintf_s(pathFormat.c_str(), ++i);
	while (std::filesystem::exists(std::filesystem::path(obsPath)));
	return i-1;
}

std::optional<ErrorMessage> parseGeneralConfigFile(const std::string &path, GeneralConfig &config)
{
	json cfg;
	auto error = readJSON(path, cfg);
	if (error) return error;

	std::filesystem::path basePath = std::filesystem::path(path).parent_path();

	JSON_PARSE_TRY_BLOCK
	{
		if (cfg.contains("simulation"))
		{
			auto &simulation = cfg["simulation"];
			if (simulation.contains("blobPxStdDev"))
				config.simulation.blobPxStdDev = simulation["blobPxStdDev"].get<float>();
			if (config.simulation.blobPxStdDev <= 0.0f)
				config.simulation.blobPxStdDev = 0.0001f;
			if (simulation.contains("trackingTargets"))
			{
				float defaultMarkerFoV = 180;
				float defaultMarkerSize = 0.005f;
				auto &targets = simulation["trackingTargets"];
				std::optional<ErrorMessage> error;
				if (targets.is_array())
				{
					for (auto &md : targets)
					{
						if (md.is_string())
							error = parseTargetObjFile(basePath / md, config.simulation.trackingTargets, defaultMarkerFoV, defaultMarkerSize);
						else if (md.is_object())
						{
							float v = md.contains("markerViewAngle") && md["markerViewAngle"].is_number()? md["markerViewAngle"].get<float>() : defaultMarkerFoV;
							float s = md.contains("markerSizeMM") && md["markerSizeMM"].is_number()? md["markerSizeMM"].get<float>() : defaultMarkerSize;
							error = parseTargetObjFile(basePath / md["path"], config.simulation.trackingTargets, v, s);
						}
						if (error) return error;
					}
				}
				else if (targets.is_string())
				{
					error = parseTargetObjFile(targets, config.simulation.trackingTargets, defaultMarkerFoV, defaultMarkerSize);
					if (error) return error;
				}
			}
			if (simulation.contains("cameras"))
			{
				auto &cameras = simulation["cameras"];
				if (cameras.is_string())
				{
					config.simulation.cameraDefPath = basePath / cameras.get<std::string>();
					auto error = parseCameraCalibrations(config.simulation.cameraDefPath, config.simulation.cameraDefinitions);
					if (error) return error;
				}
			}
		}
	}
	JSON_PARSE_CATCH_BLOCK

	return std::nullopt;
}

std::optional<ErrorMessage> storeGeneralConfigFile(const std::string &path, const GeneralConfig &config)
{
	// TODO
	return std::nullopt;
}

std::optional<ErrorMessage> parseCameraConfigFile(const std::string &path, CameraConfigMap &configMap)
{
	json calib;
	auto error = readJSON(path, calib);
	if (error) return error;

	JSON_PARSE_TRY_BLOCK
	{
		// Parse camera calibration
		if (calib.contains("configurations"))
		{
			auto &configurations = calib["configurations"];
			if (configurations.is_array())
			{
				configMap.configurations.resize(configurations.size());
				int i = 0;
				for (auto &cfg : configurations)
				{
					CameraConfig &config = configMap.configurations[i++];
					if (!cfg["label"].empty()) config.label = cfg["label"].get<std::string>();
					if (!cfg["resolutionX"].empty()) config.width = cfg["resolutionX"].get<int>();
					if (!cfg["resolutionY"].empty()) config.height = cfg["resolutionY"].get<int>();
					if (!cfg["synchronised"].empty()) config.synchronised = cfg["synchronised"].get<bool>();
					if (!cfg["framerate"].empty()) config.framerate = cfg["framerate"].get<int>();
					if (!cfg["shutterSpeed"].empty()) config.exposure = cfg["shutterSpeed"].get<int>();
					if (!cfg["gain"].empty()) config.gain = cfg["gain"].get<int>();
					if (!cfg["absThreshold"].empty()) config.blobProcessing.thresholds.absolute = cfg["absThreshold"].get<int>();
					if (!cfg["edgeThreshold"].empty()) config.blobProcessing.thresholds.edge = cfg["edgeThreshold"].get<int>();
					if (!cfg["filter"].empty()) config.filter = cfg["filter"].get<int>();
					if (config.filter != 0 || config.filter != 1) config.filter = 0; // Might have more than two in the future
					if (!cfg["enableStrobe"].empty()) config.enableStrobe = cfg["enableStrobe"].get<bool>();
					if (!cfg["strobeOffset"].empty()) config.strobeOffset = cfg["strobeOffset"].get<int>();
					if (!cfg["strobeLength"].empty()) config.strobeLength = cfg["strobeLength"].get<int>();
				}
			}
		}

		// Parse camera map
		if (calib.contains("cameras"))
		{
			auto &cameras = calib["cameras"];
			if (cameras.is_array())
			{
				for (auto &cam : cameras)
				{
					if (cam["id"].empty() || !cam["id"].is_number_integer()) continue;
					if (cam["configuration"].empty() || !cam["configuration"].is_number_integer()) continue;
					CameraID id = parseCameraID(cam["id"]);
					int cfg = cam["configuration"].get<int>();
					if (cfg >= configMap.configurations.size()) continue;
					configMap.cameraConfigs[id] = cfg;
					LOGC(LInfo, "Loaded config %d for camera %u!", cfg, id);
				}
			}
		}
	}
	JSON_PARSE_CATCH_BLOCK

	return std::nullopt;
}

std::optional<ErrorMessage> storeCameraConfigFile(const std::string &path, const CameraConfigMap &config)
{
	json file;

	// Write camera configurations
	file["configurations"] = json::array();
	for (const CameraConfig &config : config.configurations)
	{
		json cfg;
		cfg["label"] = config.label;
		cfg["resolutionX"] = config.width;
		cfg["resolutionY"] = config.height;
		cfg["synchronised"] = config.synchronised;
		cfg["framerate"] = config.framerate;
		cfg["shutterSpeed"] = config.exposure;
		cfg["gain"] = config.gain;
		cfg["absThreshold"] = config.blobProcessing.thresholds.absolute;
		cfg["edgeThreshold"] = config.blobProcessing.thresholds.edge;
		cfg["filter"] = config.filter;
		cfg["enableStrobe"] = config.enableStrobe;
		cfg["strobeOffset"] = config.strobeOffset;
		cfg["strobeLength"] = config.strobeLength;
		file["configurations"].push_back(std::move(cfg));
	}

	// Write camera map
	file["cameras"] = json::array();
	for (const auto &map : config.cameraConfigs)
	{
		json cam;
		cam["id"] = map.first;
		cam["configuration"] = map.second;
		file["cameras"].push_back(std::move(cam));
	}

	return writeJSON(path, file);
}

std::optional<ErrorMessage> parseLensPresets(const std::string &path, std::map<int, LensCalib> &lensCalib, int &defaultLens)
{
	json calib;
	auto error = readJSON(path, calib);
	if (error) return error;

	JSON_PARSE_TRY_BLOCK
	{ // Parse camera calibration
		if (calib.contains("lenses"))
		{
			auto &jsLenses = calib["lenses"];
			if (jsLenses.is_array())
			{
				int i = 0;
				for (auto &jsLens : jsLenses)
				{
					if (!jsLens.contains("id") || !jsLens.contains("distortion")) return "Missing id or distortion in lens preset!";
					LensCalib calib = {};
					if (jsLens.contains("id"))
						calib.id = jsLens["id"].get<int>();
					if (jsLens.contains("label"))
						calib.label = jsLens["label"].get<std::string>();
					if (jsLens.contains("f"))
						calib.f = jsLens["f"].get<CVScalar>();
					if (jsLens.contains("distortion"))
					{
						auto &distortion = jsLens["distortion"];
						if (distortion.is_object())
						{
							calib.k1 = distortion["k1"].get<CVScalar>();
							calib.k2 = distortion["k2"].get<CVScalar>();
							calib.k3 = distortion["k3"].get<CVScalar>();
						}
					}

					lensCalib[calib.id] = calib;
				}
			}
		}

		defaultLens = -1;
		if (calib.contains("default"))
			defaultLens = calib["default"].get<int>();
		if (!lensCalib.empty() && !lensCalib.contains(defaultLens))
			defaultLens = lensCalib.begin()->first;
	}
	JSON_PARSE_CATCH_BLOCK

	return std::nullopt;
}

std::optional<ErrorMessage> storeLensPresets(const std::string &path, const std::map<int, LensCalib> &lensCalib, int defaultLens)
{
	json file;

	// Write camera calibration
	file["lenses"] = json::array();
	for (const auto &lensIt : lensCalib)
	{
		const LensCalib &lens = lensIt.second;
		if (lens.id < 0) continue;
		json jsLens;
		// Write intrinsic calibration
		jsLens["id"] = lens.id;
		jsLens["label"] = lens.label;
		jsLens["f"] = lens.f;
		jsLens["distortion"]["k1"] = lens.k1;
		jsLens["distortion"]["k2"] = lens.k2;
		jsLens["distortion"]["k3"] = lens.k3;
		file["lenses"].push_back(std::move(jsLens));
	}

	file["default"] = defaultLens;

	return writeJSON(path, file);
}

std::optional<ErrorMessage> parseCameraCalibrations(const std::string &path, std::vector<CameraCalib> &cameraCalib)
{
	json calib;
	auto error = readJSON(path, calib);
	if (error) return error;

	JSON_PARSE_TRY_BLOCK
	{ // Parse camera calibration
		if (calib.contains("cameras"))
		{
			auto &jsCameras = calib["cameras"];
			if (jsCameras.is_array())
			{
				cameraCalib.resize(jsCameras.size(), {});
				int i = 0;
				for (auto &jsCamera : jsCameras)
				{
					CameraCalib &calib = cameraCalib[i++];

					// Parse intrinsic calibration
					if (jsCamera.contains("id"))
					{
						calib.id = parseCameraID(jsCamera["id"]);
					}
					if (jsCamera.contains("f"))
					{
						calib.f = jsCamera["f"].get<CVScalar>();
						calib.fInv = 1.0/calib.f;
					}
					else if (jsCamera.contains("fov"))
					{
						if (jsCamera["fov"].is_number_float())
						{
							CVScalar fovH = jsCamera["fov"].get<CVScalar>();
							calib.fInv = std::tan(fovH/180.0*PI/2);
							calib.f = 1.0/calib.fInv;
						}
						else if (jsCamera["fov"].is_object())
						{
							CVScalar sensorX = jsCamera["fov"]["sensorSizeX"].get<CVScalar>();
							CVScalar focalLen = jsCamera["fov"]["focalLength"].get<CVScalar>();
							calib.f = 2*focalLen/sensorX;
							calib.fInv = 1.0/calib.f;
						}
					}
					if (jsCamera.contains("principalPoint"))
					{
						auto &principalPoint = jsCamera["principalPoint"];
						if (principalPoint.is_object())
						{
							calib.principalPoint.x() = principalPoint["x"].get<CVScalar>();
							calib.principalPoint.y() = principalPoint["y"].get<CVScalar>();
						}
					}
					if (jsCamera.contains("distortion"))
					{
						auto &distortion = jsCamera["distortion"];
						if (distortion.is_object())
						{
							calib.distortion.k1 = distortion["k1"].get<CVScalar>();
							calib.distortion.k2 = distortion["k2"].get<CVScalar>();
							calib.distortion.k3 = distortion["k3"].get<CVScalar>();
							calib.distortion.p1 = distortion["p1"].get<CVScalar>();
							calib.distortion.p2 = distortion["p2"].get<CVScalar>();
						}
					}
					if (jsCamera.contains("lensID"))
					{
						calib.lensID = jsCamera["lensID"].get<int>();
					}
					// Parse extrinsic calibration
					calib.transform = Eigen::Transform<CVScalar,3,Eigen::Isometry>::Identity();
					if (jsCamera.contains("position"))
					{
						auto &position = jsCamera["position"];
						if (position.is_object())
						{
							calib.transform.translation().x() = position["x"].get<CVScalar>();
							calib.transform.translation().y() = position["y"].get<CVScalar>();
							calib.transform.translation().z() = position["z"].get<CVScalar>();
						}
					}
					if (jsCamera.contains("rotation"))
					{
						auto &rotation = jsCamera["rotation"];
						if (rotation.is_object())
						{
							Eigen::Vector3d rotEA;
							rotEA.x() = rotation["x"].get<double>();
							rotEA.y() = rotation["y"].get<double>();
							rotEA.z() = rotation["z"].get<double>();
							calib.transform.linear() = getRotationXYZ(rotEA.cast<CVScalar>() / 180.0f * PI);
						}
					}
					calib.UpdateDerived();
				}
			}
		}
	}
	JSON_PARSE_CATCH_BLOCK

	return std::nullopt;
}



std::optional<ErrorMessage> storeCameraCalibrations(const std::string &path, const std::vector<CameraCalib> &cameraCalib)
{
	json file;

	// Write camera calibration
	file["cameras"] = json::array();
	for (const CameraCalib &calib : cameraCalib)
	{
		if (calib.invalid()) continue;
		json cam;
		// Write intrinsic calibration
		cam["id"] = calib.id;
		cam["f"] = calib.f;
		cam["principalPoint"]["x"] = calib.principalPoint.x();
		cam["principalPoint"]["y"] = calib.principalPoint.y();
		cam["distortion"]["k1"] = calib.distortion.k1;
		cam["distortion"]["k2"] = calib.distortion.k2;
		cam["distortion"]["k3"] = calib.distortion.k3;
		cam["distortion"]["p1"] = calib.distortion.p1;
		cam["distortion"]["p2"] = calib.distortion.p2;
		cam["lensID"] = calib.lensID;
		// Write extrinsic calibration
		Eigen::Matrix<CVScalar,3,1> pos = calib.transform.translation();
		cam["position"]["x"] = pos.x();
		cam["position"]["y"] = pos.y();
		cam["position"]["z"] = pos.z();
		Eigen::Matrix<CVScalar,3,1> rot = getEulerXYZ(calib.transform.rotation()) / PI * 180.0;
		cam["rotation"]["x"] = rot.x();
		cam["rotation"]["y"] = rot.y();
		cam["rotation"]["z"] = rot.z();
		file["cameras"].push_back(std::move(cam));
	}

	return writeJSON(path, file);
}

static IMUCalib readIMUCalib(json &jsIMU)
{
	IMUCalib calib = {};
	if (jsIMU.contains("conversion"))
	{
		int i = 0;
		auto arr = calib.conversion.array();
		for (auto &val : jsIMU["conversion"])
			arr(i++) = val.is_number_integer()? val.get<uint8_t>() : 0;
	}
	if (jsIMU.contains("orientation"))
	{
		int i = 0;
		auto arr = calib.orientation.coeffs().array();
		for (auto &val : jsIMU["orientation"])
			arr(i++) = val.is_number_float()? val.get<float>() : NAN;
	}
	if (jsIMU.contains("offset"))
	{
		int i = 0;
		auto arr = calib.offset.array();
		for (auto &val : jsIMU["offset"])
			arr(i++) = val.is_number_float()? val.get<float>() : NAN;
	}
	if (jsIMU.contains("timestampOffsetUS") && jsIMU["timestampOffsetUS"].is_number_integer())
	{
		calib.timestampOffsetUS = jsIMU["timestampOffsetUS"].get<int>();
	}
	return calib;
}

static void writeIMUCalib(const IMUCalib &calib, json &jsIMU)
{
	{
		auto arr = calib.conversion.matrix().array();
		for (int i = 0; i < arr.size(); i++)
			jsIMU["conversion"].push_back(arr(i));
	}
	if (!calib.orientation.coeffs().hasNaN())
	{
		auto arr = calib.orientation.coeffs().array();
		for (int i = 0; i < arr.size(); i++)
			jsIMU["orientation"].push_back(arr(i));
	}
	if (!calib.offset.hasNaN())
	{
		auto arr = calib.offset.array();
		for (int i = 0; i < arr.size(); i++)
			jsIMU["offset"].push_back(arr(i));
	}
	if (calib.timestampOffsetUS != 0)
	{
		jsIMU["timestampOffsetUS"] = calib.timestampOffsetUS;
	}
}

static bool readTargetCalib(json &jsTarget, TargetCalibration3D &target)
{
	float defaultAngle = jsTarget.contains("DefaultAngle")? jsTarget["DefaultAngle"].get<float>() : 180;
	float defaultSize = (jsTarget.contains("DefaultSize")? jsTarget["DefaultSize"].get<float>() : 8) / 1000.0f;
	float defaultViewAngle = std::cos(defaultAngle/360*PI);

	if (!jsTarget.contains("markers")) return false;
	auto &jsMarkers = jsTarget["markers"];
	if (!jsMarkers.is_array()) return false;
	target.markers.reserve(jsMarkers.size());
	for (auto &jsMarker : jsMarkers)
	{
		TargetMarker marker = {};
		if (!jsMarker.is_object()) return false;
		// Read position
		if (!jsMarker.contains("Position")) return false;
		auto &pos = jsMarker["Position"];
		if (!pos.is_array() || pos.size() < 3) return false;
		marker.pos = Eigen::Vector3f(pos[0].get<float>(), pos[1].get<float>(), pos[2].get<float>());
		// Read normals
		if (!jsMarker.contains("Normal")) return false;
		auto &nrm = jsMarker["Normal"];
		if (!nrm.is_array() || nrm.size() < 3) return false;
		marker.nrm = Eigen::Vector3f(nrm[0].get<float>(), nrm[1].get<float>(), nrm[2].get<float>());
		// Read other values
		if (jsMarker.contains("Angle")) // In degrees
			marker.viewAngle = std::cos(jsMarker["Angle"].get<float>()/360*PI);
		else
			marker.viewAngle = defaultViewAngle;
		if (jsMarker.contains("Size")) // In mm
			marker.size = jsMarker["Size"].get<float>() / 1000.0f;
		else
			marker.size = defaultSize;
		target.markers.push_back(std::move(marker));
	}
	target.updateMarkers();
	return true;
}

static void writeTargetCalib(const TargetCalibration3D &target, json &jsTarget)
{
	// Write markers
	jsTarget["markers"] = json::array();
	auto &jsMarkers = jsTarget["markers"];
	for (auto &marker : target.markers)
	{
		json jsMarker;
		jsMarker["Position"] = json::array({
			marker.pos.x(),
			marker.pos.y(),
			marker.pos.z()
		});
		jsMarker["Normal"] = json::array({
			marker.nrm.x(),
			marker.nrm.y(),
			marker.nrm.z()
		});
		jsMarker["Angle"] = std::acos(marker.viewAngle)*360/PI;
		jsMarker["Size"] = marker.size * 1000.0f;
		jsMarkers.push_back(std::move(jsMarker));
	}
}

static std::optional<ErrorMessage> parseTrackerConfiguration(std::filesystem::path basePath, json jsTracker, std::optional<TrackerConfig> &trackerConfig)
{
	TrackerConfig tracker(jsTracker["id"].get<int>(),
		jsTracker["label"].get<std::string>(),
		(TrackerConfig::TrackerType)jsTracker["type"].get<int>());

	tracker.trigger = (TrackerConfig::TrackerTrigger)(jsTracker["trigger"].get<int>() & TrackerConfig::TRIGGER_MASK);
	tracker.expose = jsTracker["expose"].get<int>() < TrackerConfig::EXPOSE_MAX?
		(TrackerConfig::TrackerExpose)jsTracker["expose"].get<int>() : TrackerConfig::EXPOSE_BY_DEFAULT;

	if (tracker.type == TrackerConfig::TRACKER_TARGET)
	{
		auto &jsTarget = jsTracker["target"];

		if (jsTarget.contains("calib") && jsTarget["calib"].is_string())
		{ // Default going forward
			auto targetFile = basePath / jsTarget["calib"];
			json jsCalib;
			auto error = readJSON(targetFile, jsCalib);
			if (error) return error;
			try { readTargetCalib(jsCalib, tracker.calib); }
			catch (json::exception e)
			{
				return asprintf_s("Failed to fully parse target calib '%s': '%s'!", targetFile.c_str(), e.what());
			}
		}
		else if (!readTargetCalib(jsTarget, tracker.calib))
			return asprintf_s("Failed to parse target calib for target '%s' (%d)!", tracker.label.c_str(), tracker.id);
		if (jsTarget.contains("detection") && jsTarget["detection"].is_object())
		{
			auto &jsDetect = jsTarget["detection"];
			if (jsDetect.contains("match3D") && jsDetect["match3D"].is_boolean())
				tracker.detectionConfig.match3D = jsDetect["match3D"].get<bool>();
			if (jsDetect.contains("search2D") && jsDetect["search2D"].is_boolean())
				tracker.detectionConfig.search2D = jsDetect["search2D"].get<bool>();
			if (jsDetect.contains("probe2D") && jsDetect["probe2D"].is_boolean())
				tracker.detectionConfig.probe2D = jsDetect["probe2D"].get<bool>();
			if (jsDetect.contains("probeCount") && jsDetect["probeCount"].is_number_unsigned())
				tracker.detectionConfig.probeCount = jsDetect["probeCount"].get<int>();
		}
	}
	else if (tracker.type == TrackerConfig::TRACKER_MARKER)
	{
		if (jsTracker.contains("markerSize") && jsTracker["markerSize"].is_number_float())
			tracker.markerSize = jsTracker["markerSize"].get<float>();
	}
	else if (tracker.type == TrackerConfig::TRACKER_VIRTUAL)
	{
		if (jsTracker.contains("subTrackers") && jsTracker["subTrackers"].is_array())
		{
			auto &jsSubTrackers = jsTracker["subTrackers"];
			tracker.virtConfig.ids.reserve(jsSubTrackers.size());
			for (auto &jsSubTracker : jsSubTrackers)
			{
				tracker.virtConfig.ids.push_back(jsSubTracker.get<int>());
			}
		}
	}
	else return asprintf_s("Unknown tracker type %d!", tracker.type);

	if (jsTracker.contains("imu") && jsTracker["imu"].is_object())
	{
		auto &jsIMU = jsTracker["imu"];
		if (jsIMU.contains("driver") && jsIMU["driver"].is_number_unsigned()
			&& jsIMU.contains("id") && jsIMU["id"].is_string())
		{
			tracker.imuIdent.driver = (IMUDriver)jsIMU["driver"].get<int>();
			if (tracker.imuIdent.driver <= IMU_DRIVER_NONE || tracker.imuIdent.driver >= IMU_DRIVER_MAX)
			{
				tracker.imuIdent.driver = IMU_DRIVER_NONE;
			}
			else
			{
				tracker.imuIdent.string = jsIMU["id"].get<std::string>();
				tracker.imuCalib = readIMUCalib(jsIMU);
			}
		}
	}

	trackerConfig = std::move(tracker);
	return std::nullopt;
}

std::optional<ErrorMessage> parseTrackerConfiguration(const std::string &folder, int id, std::optional<TrackerConfig> &trackerConfig)
{
	std::filesystem::path basePath = std::filesystem::path(folder);
	std::filesystem::path path = basePath / asprintf_s("tracker_%d.json", id);

	json jsTracker;
	auto error = readJSON(path, jsTracker);
	if (error) return error;

	JSON_PARSE_TRY_BLOCK
	{
		error = parseTrackerConfiguration(basePath, jsTracker, trackerConfig);
	}
	JSON_PARSE_CATCH_BLOCK

	return error;
}

std::optional<ErrorMessage> parseTrackerConfigurations(const std::string &folder, std::vector<TrackerConfig> &trackerConfig)
{
	std::filesystem::path basePath = std::filesystem::path(folder);
	if (!std::filesystem::is_directory(basePath) && !std::filesystem::create_directory(basePath))
		return asprintf_s("Failed to create target calib directory '%s'!", basePath.c_str());

	std::optional<ErrorMessage> error;

	for (const auto &file : std::filesystem::directory_iterator(basePath))
	{
		if (file.path().extension().compare(".json") != 0) continue;
		const std::string &str = file.path().stem().string();
		if (!str.starts_with("tracker_")) continue;
		int id = -1;
		if (std::sscanf(str.data(), "tracker_%d.json", &id) != 1) continue;

		std::optional<TrackerConfig> tracker;
		error = parseTrackerConfiguration(folder, id, tracker);
		if (!error && tracker)
			trackerConfig.push_back(std::move(tracker.value()));
	}

	return error;
}

std::optional<ErrorMessage> parseTrackerConfigurationsLegacy(const std::string &path, std::vector<TrackerConfig> &trackerConfig)
{
	json file;
	auto error = readJSON(path, file);
	if (error) return error;

	JSON_PARSE_TRY_BLOCK
	{
		// Parse tracker configurations
		if (!file.contains("trackers") || !file["trackers"].is_array()) return "Invalid tracker configuration!";
		auto &jsTrackers = file["trackers"];
		trackerConfig.reserve(trackerConfig.size() + jsTrackers.size());
		for (auto &jsTracker : jsTrackers)
		{
			std::optional<TrackerConfig> tracker;
			error = parseTrackerConfiguration("", jsTracker, tracker);
			if (error) return error;
			if (tracker)
				trackerConfig.push_back(std::move(tracker.value()));
		}
	}
	JSON_PARSE_CATCH_BLOCK

	return error;
}

static std::optional<ErrorMessage> writeTrackerConfiguration(json &jsTracker, const TrackerConfig &tracker)
{
	std::optional<ErrorMessage> error;

	jsTracker["id"] = tracker.id;
	jsTracker["label"] = tracker.label;
	jsTracker["type"] = tracker.type;
	jsTracker["trigger"] = tracker.trigger;
	jsTracker["expose"] = tracker.expose;

	if (tracker.type == TrackerConfig::TRACKER_TARGET)
	{
		jsTracker["target"]["calib"] = asprintf_s("target_%d.json", tracker.id);

		auto &jsDetect = jsTracker["target"]["detection"];
		jsDetect["match3D"] = tracker.detectionConfig.match3D;
		jsDetect["search2D"] = tracker.detectionConfig.search2D;
		jsDetect["probe2D"] = tracker.detectionConfig.probe2D;
		jsDetect["probeCount"] = tracker.detectionConfig.probeCount;
	}
	else if (tracker.type == TrackerConfig::TRACKER_MARKER)
	{
		jsTracker["markerSize"] = tracker.markerSize;
	}
	else if (tracker.type == TrackerConfig::TRACKER_VIRTUAL)
	{
		json jsSubTrackers = json::array();
		for (int i = 0; i < tracker.virtConfig.ids.size(); i++)
		{
			jsSubTrackers.push_back(tracker.virtConfig.ids[i]);
		}
		jsTracker["subTrackers"] = jsSubTrackers;
	}
	else error = asprintf_s("Unknown tracker type %d!", tracker.type);

	if (tracker.imuIdent)
	{
		auto &jsIMU = jsTracker["imu"];
		jsIMU["driver"] = tracker.imuIdent.driver;
		jsIMU["id"] = tracker.imuIdent.string;
		writeIMUCalib(tracker.imuCalib, jsIMU);
	}

	return error;
}

std::optional<ErrorMessage> storeTrackerConfiguration(const std::string &folder, TrackerConfig &tracker)
{
	if (tracker.isSimulated) return std::nullopt;
	std::filesystem::path basePath = std::filesystem::path(folder);
	if (!std::filesystem::is_directory(basePath) && !std::filesystem::create_directory(basePath))
		return asprintf_s("Failed to create target calib directory '%s'!", basePath.c_str());

	std::optional<ErrorMessage> error;

	std::filesystem::path trackerFile = basePath / asprintf_s("tracker_%d.json", tracker.id);
	if (tracker.configDirty || !std::filesystem::is_regular_file(trackerFile))
	{
		json jsTracker;
		error = writeTrackerConfiguration(jsTracker, tracker);
		if (!error)
		{
			error = writeJSON(trackerFile, jsTracker);
			if (!error) tracker.configDirty = false;
		}
	}

	if (error) return error;

	if (tracker.type == TrackerConfig::TRACKER_TARGET)
	{
		std::filesystem::path targetFile = basePath / asprintf_s("target_%d.json", tracker.id);
		if (tracker.targetDirty || !std::filesystem::is_regular_file(targetFile))
		{
			json jsTarget;
			writeTargetCalib(tracker.calib, jsTarget);
			error = writeJSON(targetFile, jsTarget);
			if (!error) tracker.targetDirty = false;
		}
	}
	
	return error;
}

std::optional<ErrorMessage> storeTrackerConfigurations(const std::string &folder, std::vector<TrackerConfig> &trackerConfig)
{
	for (TrackerConfig &tracker : trackerConfig)
	{
		auto error = storeTrackerConfiguration(folder, tracker);
		if (error) return error;
	}
	return std::nullopt;
}

std::optional<ErrorMessage> parseRecording(const std::string &path, std::vector<CameraConfigRecord> &cameras, TrackingRecord &record, std::size_t &frameOffset, bool separate)
{
	std::filesystem::path imgFolder = std::filesystem::path(path).replace_extension();
	frameOffset = 0;

	json file;
	auto error = readJSON(path, file);
	if (error) return error;

	JSON_PARSE_TRY_BLOCK
	{
		if (!file.contains("frameRecords") || !file["frameRecords"].is_object())
			return asprintf_s("Invalid recording file '%s' - frameRecords!", path.c_str());
		auto &jsRecords = file["frameRecords"];

		if (!jsRecords.contains("frames") || !jsRecords["frames"].is_array())
			return asprintf_s("Invalid recording file '%s' - frames!", path.c_str());
		auto &jsFrames = jsRecords["frames"];

		std::vector<CameraConfigRecord> parsedCameras;
		if (jsRecords.contains("cameras") && jsRecords["cameras"].is_array())
		{
			auto &jsCameras = jsRecords["cameras"];
			parsedCameras.reserve(jsCameras.size());
			for (auto &jsCamera : jsCameras)
			{
				if (jsCamera.is_object())
				{
					if (!jsCamera.contains("id")) return asprintf_s("Invalid recording file '%s' - camera ID!", path.c_str());
					parsedCameras.emplace_back(parseCameraID(jsCamera["id"]),
						jsCamera.contains("frameX")? jsCamera["frameX"].get<int>() : 1280,
						jsCamera.contains("frameY")? jsCamera["frameY"].get<int>() : 800
					);
				}
				else if (jsCamera.is_number_integer())
					parsedCameras.emplace_back(parseCameraID(jsCamera), 1280, 800);
				else
					return asprintf_s("Invalid recording file '%s' - camera ID!", path.c_str());
			}
		}
		else if (jsRecords.contains("cameraCount") && jsRecords["cameraCount"].is_number_integer())
		{
			parsedCameras.reserve(jsRecords["cameraCount"].get<int>());
			for (int i = 0; i < jsRecords["cameraCount"].get<int>(); i++)
				parsedCameras.emplace_back(i+1, 1280, 800);
		}
		else
			return asprintf_s("Invalid recording file '%s' - cameras!", path.c_str());

		std::vector<int> cameraIndices(parsedCameras.size(), -1);
		if (separate)
		{
			for (int c = 0; c < parsedCameras.size(); c++)
			{
				cameraIndices[c] = cameras.size();
				cameras.push_back(parsedCameras[c]);
			}
		}
		else if (cameras.empty())
		{
			std::iota(cameraIndices.begin(), cameraIndices.end(), 0);
			cameras = parsedCameras;
		}
		else
		{ // Check match with any prior cameras (for segmented/appended recordings)
			for (int c = 0; c < parsedCameras.size(); c++)
			{
				// Try to match with existing camera
				for (int cc = 0; cc < cameras.size(); cc++)
					if (cameras[cc].ID == parsedCameras[c].ID)
						cameraIndices[c] = cc;
				if (cameraIndices[c] < 0)
				{ // Add as new camera
					cameraIndices[c] = cameras.size();
					cameras.push_back(parsedCameras[c]);
				}
			}
		}

		// TODO: Implement some kind of lock_write in BlockedQueue that locks any write attempts but allows for getView to return old state?
		TimePoint_t startT;
		bool foundFirst = false;
		auto ref_now = getAccurateClockReference<std::chrono::system_clock::time_point, TimePoint_t>();
		for (auto &jsFrame : jsFrames)
		{
			if (!jsFrame.contains("cameras")) continue;
			if (!jsFrame["cameras"].is_array()) continue;
			auto &jsCameras = jsFrame["cameras"];

			FrameNum num = jsFrame["num"].get<FrameNum>();
			if (!foundFirst)
			{ // Map it to the next index
				auto frames = record.frames.getView();
				frameOffset = num - frames.endIndex();
				startT = frames.empty()? sclock::now() : frames.back()->time;
				foundFirst = true;
			}
			auto frame = std::make_shared<FrameRecord>();
			frame->ID = jsFrame["id"].get<FrameID>();
			frame->num = num-frameOffset;
			frame->time = startT + std::chrono::microseconds(jsFrame["dt"].get<uint64_t>());
			frame->timeUTC = convertClockWithRef(frame->time, ref_now);
			frame->cameras.resize(cameras.size());
			if (jsCameras.size() > parsedCameras.size())
				return asprintf_s("Invalid recording file '%s' - frame with more cameras!", path.c_str());

			for (int c = 0; c < jsCameras.size(); c++)
			{
				auto &jsCamera = jsCameras[c];
				auto &camera = frame->cameras[cameraIndices[c]];

				camera.received = jsCamera.contains("blobs") && jsCamera["blobs"].is_array();
				if (camera.received)
				{
					auto &jsBlobs = jsCamera["blobs"];
					camera.rawPoints2D.reserve(jsBlobs.size());
					camera.properties.reserve(jsBlobs.size());

					for (auto &jsBlob : jsBlobs)
					{
						camera.rawPoints2D.emplace_back(jsBlob["x"].get<float>(), jsBlob["y"].get<float>());
						camera.properties.emplace_back(
							jsBlob["s"].get<float>(),
							jsBlob.contains("v")? jsBlob["v"].get<int>() : 1000
						);
					}
				}

				if (!jsCamera.contains("image")) continue;
				json jsImage = jsCamera["image"];

				std::vector<uint8_t> jpeg;
				{ // Try to load jpeg image
					auto imgPath = imgFolder / jsImage["path"].get<std::string>();
					std::ifstream fif(imgPath, std::ios::binary);
					if (!fif.is_open()) continue;
					fif.unsetf(std::ios::skipws);
					fif.seekg(0, std::ios::end);
					jpeg.reserve(fif.tellg());
					fif.seekg(0, std::ios::beg);
					jpeg.insert(jpeg.begin(),
							std::istream_iterator<uint8_t>(fif),
							std::istream_iterator<uint8_t>());
				}

				// Record image
				camera.image = std::make_shared<CameraImageRecord>();
				auto &image = *camera.image;
				int camIndex = frame->cameras.size()-1;
				image.cameraID = cameras[camIndex].ID;
				image.frameID = frame->ID;
				image.jpeg = std::move(jpeg);

				image.frameX = jsImage.contains("frameX")? jsImage["frameX"].get<int>() : cameras[camIndex].frameX;
				image.frameY = jsImage.contains("frameY")? jsImage["frameY"].get<int>() : cameras[camIndex].frameY;

				if (jsImage.contains("min") && jsImage.contains("max"))
				{
					image.boundsPx.min.x() = jsImage["min"]["x"].get<int>();
					image.boundsPx.min.y() = jsImage["min"]["y"].get<int>();
					image.boundsPx.max.x() = jsImage["max"]["x"].get<int>();
					image.boundsPx.max.y() = jsImage["max"]["y"].get<int>();
				}
				else
					image.boundsPx = Bounds2i(0, 0, image.frameX, image.frameY);

				image.imageX = jsImage.contains("imageX")? jsImage["imageX"].get<int>() : image.boundsPx.extends().x();
				image.imageY = jsImage.contains("imageY")? jsImage["imageY"].get<int>() : image.boundsPx.extends().y();
			}

			record.frames.insert(frame->num, std::move(frame));
		}

		if (!file.contains("imuRecords")) return std::nullopt;
		if (!file["imuRecords"].is_array()) return std::nullopt;
		auto &jsIMUs = file["imuRecords"];
		auto noMag = nlohmann::json::array({ 0.0f, 0.0f, 0.0f });
		record.imus.reserve(jsIMUs.size());
		for (auto &jsIMU : jsIMUs)
		{
			IMUIdent id;
			id.driver = (IMUDriver)jsIMU["driver"].get<int>();
			id.string = jsIMU.contains("id")? jsIMU["id"].get<std::string>()
				: (jsIMU.contains("device")? asprintf_s("IMU %d", jsIMU["device"].get<int>()) : "Unknown IMU");
			std::shared_ptr<IMURecord> imu = nullptr;
			for (auto &imuRecord : record.imus)
				if (imuRecord->id == id)
					imu = std::static_pointer_cast<IMURecord>(imuRecord);
			if (!imu)
			{ // New IMU
				imu = std::make_shared<IMURecord>();
				imu->id = std::move(id);
				imu->hasMag = jsIMU["hasMag"].get<bool>();
				imu->isFused = jsIMU["isFused"].get<bool>();
				record.imus.push_back(imu);
			}

			auto &jsSamples = jsIMU["samples"];
			if (imu->isFused)
			{
				for (auto &jsSample : jsSamples)
				{
					auto quat = jsSample["quat"];
					auto accel = jsSample["accel"];
					imu->samplesFused.push_back({ 
						startT + std::chrono::microseconds(jsSample["dt"].get<uint64_t>()),
						Eigen::Quaternionf(quat[3].get<float>(), quat[0].get<float>(), quat[1].get<float>(), quat[2].get<float>()),
						Eigen::Vector3f(accel[0].get<float>(), accel[1].get<float>(), accel[2].get<float>())
					});
				}
			}
			else
			{
				for (auto &jsSample : jsSamples)
				{
					auto gyro = jsSample["gyro"];
					auto accel = jsSample["accel"];
					auto mag = imu->hasMag? jsSample["mag"] : noMag;
					imu->samplesRaw.push_back({ 
						startT + std::chrono::microseconds(jsSample["dt"].get<uint64_t>()),
						Eigen::Vector3f(gyro[0].get<float>(), gyro[1].get<float>(), gyro[2].get<float>()),
						Eigen::Vector3f(accel[0].get<float>(), accel[1].get<float>(), accel[2].get<float>()),
						Eigen::Vector3f(mag[0].get<float>(), mag[1].get<float>(), mag[2].get<float>())
					});
				}
			}
		}
	}
	JSON_PARSE_CATCH_BLOCK

	return std::nullopt;
}

std::optional<ErrorMessage> saveRecording(const std::string &path, const std::vector<CameraConfigRecord> &cameras, const TrackingRecord &record, std::size_t begin, std::size_t end)
{
	// Check frame range
	auto frames = record.frames.getView(); 
	begin = std::max(begin, frames.beginIndex());
	end = std::min(end, frames.endIndex());
	if (begin >= end) return "Failed to write recording with invalid frame range!";
	auto frameBegin = frames.pos(begin);
	while (frameBegin < frames.end() && !*frameBegin) frameBegin++;
	auto frameBack = frames.pos(end-1);
	while (frameBack > frames.begin() && !*frameBack) frameBack--;
	auto frameEnd = std::next(frameBack);
	if (frameBegin >= frameEnd) return "Failed to write recording with non-existant frame range!";

	std::filesystem::path imgFolder = std::filesystem::path(path).replace_extension();

	json file;

	// Write camera map
	file["frameRecords"]["cameras"] = json::array();
	auto &jsCameras = file["frameRecords"]["cameras"];
	for (auto &camera : cameras)
	{
		json jsCamera;
		jsCamera["id"] = camera.ID;
		jsCamera["frameX"] = camera.frameX;
		jsCamera["frameY"] = camera.frameY;
		jsCameras.push_back(std::move(jsCamera));
	}

	// Time range
	TimePoint_t timeFirst = (frameBegin == frames.begin()? frameBegin : std::prev(frameBegin))->get()->time;
	TimePoint_t timeLast = frameBack->get()->time;

	// Write observations
	file["frameRecords"]["frames"] = json::array();
	auto &jsFrames = file["frameRecords"]["frames"];
	for (auto frameIt = frameBegin; frameIt != frameEnd; frameIt++)
	{
		if (!frameIt->get()) continue; // No frame recorded, can happen due to record delay (first frames), or dropped frames from hardware
		const FrameRecord &frame = *frameIt->get();
		json jsFrame;
		jsFrame["id"] = frame.ID;
		jsFrame["num"] = frame.num;
		jsFrame["dt"] = dtUS(timeFirst, frame.time);
		jsFrame["cameras"] = json::array();
		int camIndex = 0;
		for (auto &camera : frame.cameras)
		{
			json jsCamera;
			if (camera.received)
			{
				jsCamera["blobs"] = json::array();
				json &jsBlobs = jsCamera["blobs"];
				assert(camera.rawPoints2D.size() == camera.properties.size());
				for (int b = 0; b < camera.rawPoints2D.size(); b++)
				{
					json jsBlob;
					jsBlob["x"] = camera.rawPoints2D[b].x();
					jsBlob["y"] = camera.rawPoints2D[b].y();
					jsBlob["s"] = camera.properties[b].size;
					jsBlob["v"] = camera.properties[b].value;
					jsBlobs.push_back(std::move(jsBlob));
				}
			}
			if (camera.image)
			{
				auto &image = *camera.image;
				std::filesystem::create_directories(imgFolder);
				std::string imgName = asprintf_s("%u_%d.jpg", frame.ID, camIndex);
				std::ofstream fif(imgFolder / imgName, std::ios::out | std::ios::binary);
				if (fif.is_open())
				{ // TODO: Check for disk space before continuing to write
					// Though images really don't take too much space
					fif.write((const char*)image.jpeg.data(), image.jpeg.size());
					fif.close();
					json jsImage;
					jsImage["path"] = imgName;
					if (image.frameX != cameras[camIndex].frameX || image.frameY != cameras[camIndex].frameY)
					{ // Changed mode?
						jsImage["frameX"] = image.frameX;
						jsImage["frameY"] = image.frameY;
					}

					if (image.imageX != image.frameX || image.imageY != image.frameY)
					{ // Subsection of the image
						jsImage["min"]["x"] = image.boundsPx.min.x();
						jsImage["min"]["y"] = image.boundsPx.min.y();
						jsImage["max"]["x"] = image.boundsPx.max.x();
						jsImage["max"]["y"] = image.boundsPx.max.y();
						jsImage["imageX"] = image.imageX;
						jsImage["imageY"] = image.imageY;
					}
					jsCamera["image"] = std::move(jsImage);
				}
			}
			jsFrame["cameras"].push_back(std::move(jsCamera));
			camIndex++;
		}
		jsFrames.push_back(std::move(jsFrame));
	}

	// Write imu records
	file["imuRecords"] = json::array();
	auto &jsIMUs = file["imuRecords"];
	for (auto &imu : record.imus)
	{
		json jsIMU;

		if (imu->isFused)
		{
			auto samples = imu->samplesFused.getView();
			auto itBegin = std::lower_bound(samples.begin(), samples.end(), timeFirst);
			// Include some samples before start
			for (int i = 0; i < 10 && itBegin != samples.begin(); i++, itBegin--);
			auto itEnd = std::upper_bound(samples.begin(), samples.end(), timeLast);
			if (itBegin > itEnd) continue; // Had no sample in frame range

			jsIMU["samples"] = json::array();
			auto &jsSamples = jsIMU["samples"];
			for (auto it = itBegin; it != itEnd; it++)
			{
				json jsSample;
				jsSample["dt"] = dtUS(timeFirst, it->timestamp); // May be negative at beginning
				jsSample["quat"] = json::array({ it->quat.x(), it->quat.y(), it->quat.z(), it->quat.w() });
				jsSample["accel"] = json::array({ it->accel.x(), it->accel.y(), it->accel.z() });
				jsSamples.push_back(std::move(jsSample));
			}
		}
		else
		{
			auto samples = imu->samplesRaw.getView();
			auto itBegin = std::lower_bound(samples.begin(), samples.end(), timeFirst);
			// Include some samples before start
			for (int i = 0; i < 10 && itBegin != samples.begin(); i++, itBegin--);
			auto itEnd = std::upper_bound(samples.begin(), samples.end(), timeLast);
			if (itBegin > itEnd) continue; // Had no sample in frame range

			jsIMU["samples"] = json::array();
			auto &jsSamples = jsIMU["samples"];
			for (auto it = itBegin; it != itEnd; it++)
			{
				json jsSample;
				jsSample["dt"] = dtUS(timeFirst, it->timestamp); // May be negative at beginning
				jsSample["gyro"] = json::array({ it->gyro.x(), it->gyro.y(), it->gyro.z() });
				jsSample["accel"] = json::array({ it->accel.x(), it->accel.y(), it->accel.z() });
				if (imu->hasMag)
					jsSample["mag"] = json::array({ it->mag.x(), it->mag.y(), it->mag.z() });					
				jsSamples.push_back(std::move(jsSample));
			}
		}

		jsIMU["driver"] = imu->id.driver;
		jsIMU["id"] = imu->id.string;
		jsIMU["hasMag"] = imu->hasMag;
		jsIMU["isFused"] = imu->isFused;

		jsIMUs.push_back(std::move(jsIMU));
	}

	return writeJSON(path, file);
}

std::optional<ErrorMessage> parseTrackingResults(std::string &path, TrackingRecord &record, std::size_t frameOffset)
{
	// Modify path (expecting XX_capture***.json)
	std::size_t index = path.rfind("_capture");
	if (index != std::string::npos)
		path.replace(index, 8, "_tracking");

	json file;
	auto error = readJSON(path, file);
	if (error) return error;

	JSON_PARSE_TRY_BLOCK
	{
		auto frames = record.frames.getView();
		if (!file.contains("trackingResults") || !file["trackingResults"].is_object())
			return asprintf_s("Invalid tracking file '%s' - tracking results!", path.c_str());
		auto &jsRecords = file["trackingResults"];

		if (!jsRecords.contains("frames") || !jsRecords["frames"].is_array())
			return asprintf_s("Invalid tracking file '%s' - frames!", path.c_str());
		auto &jsFrames = jsRecords["frames"];

		for (auto &jsFrame : jsFrames)
		{
			if (!jsFrame.contains("num")) continue;
			if (!jsFrame.contains("targets")) continue;
			if (!jsFrame["targets"].is_array()) continue;
			auto &jsTrackers = jsFrame["targets"];

			FrameNum num = jsFrame["num"].get<FrameNum>();
			if (num-frameOffset < frames.beginIndex()) continue;
			if (num-frameOffset >= frames.endIndex()) break;

			auto &framePtr = frames[num-frameOffset];
			if (!framePtr) continue;
			FrameRecord &frame = *framePtr;
			frame.triangulations.clear();
			// TODO: Properly integrate triangulation records (3/3)
			frame.trackers.clear();
			for (auto &jsTarget : jsTrackers)
			{
				if (!jsTarget.is_object() || !jsTarget.contains("pose") || !jsTarget["pose"].is_array() || jsTarget["pose"].size() != 4*4) continue;
				frame.trackers.push_back({});
				auto &tracker = frame.trackers.back();
				tracker.id = jsTarget["id"].get<int>();
				if (jsTarget.contains("trk"))
					tracker.result = jsTarget["trk"].get<bool>()? TrackingResult::IS_TRACKED : TrackingResult::IS_FAILURE;
				else if (jsTarget.contains("res"))
					tracker.result = jsTarget["res"].get<int>();
				else
					tracker.result = TrackingResult::NONE;
				if (jsTarget.contains("time"))
					tracker.procTimeMS = jsTarget["time"].get<float>();
				if (jsTarget.contains("mistrust"))
					tracker.mistrust = jsTarget["mistrust"].get<float>();
				int i = 0;
				auto poseArr = tracker.poseObserved.matrix().array();
				for (auto &val : jsTarget["pose"])
					poseArr(i++) = val.is_number_float()? val.get<float>() : NAN;
				tracker.error.samples = jsTarget["num"].get<int>();
				tracker.error.mean = jsTarget["avg"].get<float>();
				tracker.error.stdDev = jsTarget.contains("dev")? jsTarget["dev"].get<float>() : 0.0f;
				tracker.error.max = jsTarget["max"].get<float>();
			}
		}
	}
	JSON_PARSE_CATCH_BLOCK

	return std::nullopt;
}

std::optional<ErrorMessage> saveTrackingResults(std::string &path, const TrackingRecord &record, std::size_t begin, std::size_t end, std::size_t frameOffset)
{
	// Check frame range
	auto frames = record.frames.getView(); 
	begin = std::max(begin, frames.beginIndex());
	end = std::min(end, frames.endIndex());
	if (begin >= end) return "Failed to write tracking results with invalid frame range!";
	auto frameBegin = frames.pos(begin);
	auto frameEnd = frames.pos(end);
	if (frameBegin >= frameEnd) return "Failed to write tracking results with non-existant frame range!";

	// Modify path (expecting XX_capture***.json)
	std::size_t index = path.rfind("_capture");
	if (index != std::string::npos)
		path.replace(index, 8, "_tracking");

	json file;

	std::set<int> targetIDs;

	// Write observations
	file["trackingResults"] = json::object();
	auto &jsRecords = file["trackingResults"];
	jsRecords["frames"] = json::array();
	auto &jsFrames = jsRecords["frames"];
	for (auto frameIt = frameBegin; frameIt != frameEnd; frameIt++)
	{
		if (!frameIt->get()) continue; // No frame recorded, can happen due to record delay (first frames), or dropped frames from hardware
		const FrameRecord &frame = *frameIt->get();
		json jsFrame;
		jsFrame["id"] = frame.ID;
		jsFrame["num"] = frameOffset + frame.num;
		jsFrame["targets"] = json::array();
		for (const auto &target : frame.trackers)
		{
			json jsTarget;
			jsTarget["id"] = target.id;
			jsTarget["res"]	= (int)target.result;
			jsTarget["time"] = target.procTimeMS;
			jsTarget["mistrust"] = target.mistrust;
			jsTarget["pose"] = json::array();
			auto poseArr = target.poseObserved.matrix().array();
			for (int i = 0; i < 16; i++)
				jsTarget["pose"].push_back(poseArr(i));
			jsTarget["num"] = target.error.samples;
			jsTarget["avg"] = target.error.mean;
			jsTarget["dev"] = target.error.stdDev;
			jsTarget["max"] = target.error.max;
			jsFrame["targets"].push_back(std::move(jsTarget));
			targetIDs.insert(target.id);
		}
		jsFrames.push_back(std::move(jsFrame));
	}

	// Write trackers tracked during frame span
	jsRecords["trackers"] = json::array();
	for (int tgtID : targetIDs)
		jsRecords["trackers"].push_back(tgtID);

	if (targetIDs.empty() || jsFrames.empty())
		return std::nullopt; // Nothing to save, no error

	return writeJSON(path, file);
}

std::optional<ErrorMessage> parseSequenceDatabase(const std::string &path, std::vector<CameraID> &cameraIDs, SequenceData &sequences)
{
	json file;
	auto error = readJSON(path, file);
	if (error) return error;

	sequences = {};

	JSON_PARSE_TRY_BLOCK
	{
		if (!file.contains("observations") || !file["observations"].is_object())
			return asprintf_s("Invalid marker observations file '%s' - observations!", path.c_str());
		auto &jsObs = file["observations"];

		if (!jsObs.contains("markers") || !jsObs["markers"].is_array())
			return asprintf_s("Invalid marker observations file '%s' - markers!", path.c_str());
		auto &jsMarkers = jsObs["markers"];

		int cameraCount = 0;
		if (jsObs.contains("cameras") && jsObs["cameras"].is_array())
		{
			auto &jsCameras = jsObs["cameras"];
			cameraCount = jsCameras.size();
			for (auto &jsCam : jsCameras)
				cameraIDs.push_back(jsCam.get<CameraID>());
		}
		else if (jsObs.contains("cameraCount") && jsObs["cameraCount"].is_number_integer())
		{
			cameraCount = jsObs["cameraCount"].get<int>();
			for (int i = 0; i < cameraCount; i++)
				cameraIDs.push_back(i+1);
		}
		else
			return asprintf_s("Invalid marker observations file '%s' - no cameras!", path.c_str());
		sequences.temporary.resize(cameraCount);

		sequences.markers.reserve(jsMarkers.size());
		int m = 0;
		for (auto &jsMarker : jsMarkers)
		{
			if (!jsMarker.contains("cameras")) continue;
			if (!jsMarker["cameras"].is_array()) continue;
			auto &jsCameras = jsMarker["cameras"];
			if (jsCameras.size() > cameraCount)
				return asprintf_s("Invalid observation file '%s' - marker with more cameras!", path.c_str());

			MarkerSequences marker = {};
			marker.cameras.resize(cameraCount);

			int c = 0;
			for (auto &jsCamera : jsCameras)
			{
				if (!jsCamera.contains("sequences")) break;
				if (!jsCamera["sequences"].is_array()) break;
				auto &jsSequences = jsCamera["sequences"];

				auto &camera = marker.cameras[c++];
				camera.sequences.resize(jsSequences.size());

				int s = 0;
				for (auto &jsSequence : jsSequences)
				{
					if (!jsSequence.contains("start")) continue;
					if (!jsSequence["start"].is_number_integer()) continue;
					camera.sequences[s].startFrame = jsSequence["start"].get<int>();

					if (!jsSequence.contains("blobs")) continue;
					if (!jsSequence["blobs"].is_array()) continue;
					auto &jsBlobs = jsSequence["blobs"];
					auto &points = camera.sequences[s].points;
					auto &rawPoints = camera.sequences[s].rawPoints;
					points.reserve(jsBlobs.size());
					rawPoints.reserve(jsBlobs.size());
					for (auto &jsBlob : jsBlobs)
					{
						points.emplace_back(jsBlob["x"].get<float>(), jsBlob["y"].get<float>());
						rawPoints.emplace_back(jsBlob["rx"].get<float>(), jsBlob["ry"].get<float>());
					}

					marker.lastFrame = std::max(marker.lastFrame, camera.sequences[s].lastFrame());
					s++;
				}
			}
			if (c != cameraCount)
			{ // Error parsing marker
				continue;
			}

			sequences.lastRecordedFrame = std::max(sequences.lastRecordedFrame, marker.lastFrame);
			sequences.markers.push_back(std::move(marker));
		}
	}
	JSON_PARSE_CATCH_BLOCK

	return std::nullopt;
}

std::optional<ErrorMessage> dumpSequenceDatabase(const std::string &path, const SequenceData &sequences, const std::vector<CameraID> &cameraIDs)
{
	if (sequences.markers.size() == 0)
		return "No observation data to store!";

	json file;

	// Write camera map
	file["observations"]["cameras"] = json::array();
	auto &cameraMap = file["observations"]["cameras"];
	for (CameraID id : cameraIDs)
		cameraMap.push_back(id);

	// Write observations
	file["observations"]["markers"] = json::array();
	auto &jsMarkers = file["observations"]["markers"];
	for (auto &marker : sequences.markers)
	{
		if (marker.cameras.size() > cameraIDs.size())
			continue;
		json jsMarker;
		jsMarker["cameras"] = json::array();
		for (auto &camObs : marker.cameras)
		{
			json jsCamera;
			jsCamera["sequences"] = json::array();
			json &jsSequences = jsCamera["sequences"];
			for (auto &sequence : camObs.sequences)
			{
				json jsSequence;
				jsSequence["start"] = sequence.startFrame;
				jsSequence["blobs"] = json::array();
				json &jsBlobs = jsSequence["blobs"];
				for (int i = 0; i < sequence.points.size(); i++)
				{
					json jsBlob;
					jsBlob["x"] = sequence.points[i].x();
					jsBlob["y"] = sequence.points[i].y();
					jsBlob["rx"] = sequence.rawPoints[i].x();
					jsBlob["ry"] = sequence.rawPoints[i].y();
					jsBlobs.push_back(std::move(jsBlob));
				}
				jsSequences.push_back(std::move(jsSequence));
			}
			jsMarker["cameras"].push_back(std::move(jsCamera));
		}
		jsMarkers.push_back(std::move(jsMarker));
	}

	return writeJSON(path, file);
}

static bool parseOptTarget(json &in, ObsTarget &target)
{
	if (!in.contains("frames")) return false;
	if (!in["frames"].is_array()) return false;
	auto &jsFrames = in["frames"];

	if (!in.contains("markers")) return false;
	if (!in["markers"].is_array()) return false;
	auto &jsMarkers = in["markers"];

	if (!in.contains("markerMap")) return false;
	if (!in["markerMap"].is_array()) return false;
	auto &jsMarkerMap = in["markerMap"];

	target.markers.reserve(jsMarkers.size());
	for (auto &jsMarker : jsMarkers)
		target.markers.emplace_back(jsMarker["x"].get<float>(), jsMarker["y"].get<float>(), jsMarker["z"].get<float>());

	for (auto &jsMap : jsMarkerMap)
		target.markerMap[jsMap["first"].get<int>()] = jsMap["second"].get<int>();

	target.frames.reserve(jsFrames.size());
	for (auto &jsFrame : jsFrames)
	{
		if (!jsFrame.contains("frame")) continue;
		uint32_t frameNum = jsFrame["frame"].get<uint32_t>();

		if (!jsFrame.contains("pose")) continue;
		if (!jsFrame["pose"].is_array()) continue;
		auto &jsPose = jsFrame["pose"];
		if (jsPose.size() != 4*4) continue;

		ObsTargetFrame frame = {};
		frame.frame = frameNum;

		int i = 0;
		auto poseArr = frame.pose.matrix().array();
		for (auto &val : jsPose)
			poseArr(i++) = val.get<float>();

		// NOTE: No need to store samples, use updateTargetObservations instead

		target.frames.push_back(frame);
	}
	return true;
}

static void storeOptTarget(json &out, const ObsTarget &target)
{
	out["markers"] = json::array();
	for (auto &marker : target.markers)
	{
		json jsMarker;
		jsMarker["x"] = marker.x();
		jsMarker["y"] = marker.y();
		jsMarker["z"] = marker.z();
		out["markers"].push_back(std::move(jsMarker));
	}

	out["markerMap"] = json::array();
	for (auto &map : target.markerMap)
	{
		json jsMap;
		jsMap["first"] = map.first;
		jsMap["second"] = map.second;
		out["markerMap"].push_back(std::move(jsMap));
	}

	out["frames"] = json::array();
	for (auto &frame : target.frames)
	{
		json jsFrame;
		jsFrame["frame"] = frame.frame;

		jsFrame["pose"] = json::array();
		auto poseArr = frame.pose.matrix().array();
		for (int i = 0; i < 16; i++)
			jsFrame["pose"].push_back(poseArr(i));

		// NOTE: No need to store samples, use updateTargetObservations instead

		out["frames"].push_back(std::move(jsFrame));
	}
}

std::optional<ErrorMessage> parseTargetViewRecords(const std::string &path,
	const BlockedQueue<std::shared_ptr<FrameRecord>> &frameRecords,
	std::vector<std::shared_ptr<TargetView>> &views)
{
	json file;
	auto error = readJSON(path, file);
	if (error) return error;

	JSON_PARSE_TRY_BLOCK
	{
		if (!file.contains("views")) return asprintf_s("Invalid target views file '%s' - views!", path.c_str());
		if (!file["views"].is_array()) return asprintf_s("Invalid target views file '%s' - views!", path.c_str());
		auto &viewArr = file["views"];

		views.reserve(viewArr.size());
		std::size_t recordCount = frameRecords.getView().size();
		for (auto &view : viewArr)
		{
			ObsTarget target;
			if (!parseOptTarget(view, target)) continue;

			std::shared_ptr<TargetView> tgtView = std::make_shared<TargetView>();
			tgtView->state.calibrated = true;
			tgtView->selected = view.contains("selected")? view["selected"].get<bool>() : true;
			tgtView->id = views.size();
			tgtView->beginFrame = target.frames.front().frame;
			tgtView->endFrame = target.frames.back().frame;
			if (tgtView->endFrame >= recordCount) continue;
			tgtView->calib.contextualLock()->initialise(target.markers);
			*tgtView->target.contextualLock() = std::move(target);
			views.push_back(std::move(tgtView));
		}
	}
	JSON_PARSE_CATCH_BLOCK

	return std::nullopt;
}

std::optional<ErrorMessage> dumpTargetViewRecords(const std::string &path, const std::vector<std::shared_ptr<TargetView>> &views)
{
	json file;

	// Write views
	file["views"] = json::array();

	for (const auto &tgtView : views)
	{
		json view;
		storeOptTarget(view, *tgtView->target.contextualRLock());
		view["selected"] = tgtView->selected;
		file["views"].push_back(std::move(view));
	}

	return writeJSON(path, file);
}

std::optional<ErrorMessage> parseTargetAssemblyStage(const std::string &path, TargetAssemblyBase &base)
{
	json file;
	auto error = readJSON(path, file);
	if (error) return error;

	JSON_PARSE_TRY_BLOCK
	{
		if (!file.contains("initialViewID") || !file["initialViewID"].is_number_integer())
			return asprintf_s("Invalid target assembly stage '%s' - initialViewID!", path.c_str());
		base.initialViewID = file["initialViewID"].get<int>();
		base.assignedTrackerID = file.contains("assignedTrackerID")? file["assignedTrackerID"].get<int>() : 0;

		if (!file.contains("merged") || !file["merged"].is_array())
			return asprintf_s("Invalid target assembly stage '%s' - merged!", path.c_str());
		base.merged.reserve(file["merged"].size());
		for (auto &m : file["merged"])
			base.merged.push_back(m.get<int>());

		if (!file.contains("target") || !file["target"].is_object()
			|| !parseOptTarget(file["target"], base.target))
			return asprintf_s("Invalid target assembly stage '%s' - target!", path.c_str());

	}
	JSON_PARSE_CATCH_BLOCK

	return std::nullopt;
}

std::optional<ErrorMessage> dumpTargetAssemblyStage(const std::string &path, const TargetAssemblyBase &base)
{
	json file;
	file["initialViewID"] = base.initialViewID;
	file["assignedTrackerID"] = base.assignedTrackerID;

	// Write merged target list
	file["merged"] = json::array();
	for (int merged : base.merged)
		file["merged"].push_back(merged);

	// Write target with markers, sequence mapping, frames and their poses
	storeOptTarget(file["target"], base.target);

	return writeJSON(path, file);
}

std::optional<ErrorMessage> parseTargetObjFile(const std::string &path, std::map<std::string, TargetCalibration3D> &targets, float fov, float size)
{
	std::vector<Eigen::Vector3f> verts;
	std::vector<Eigen::Vector3f> nrms;

	std::ifstream fs(path);
	if (!fs.is_open()) return asprintf_s("Failed to open '%s' for reading!", path.c_str());

	std::map<std::string, TargetCalibration3D> groups = { { path, TargetCalibration3D() } };
	TargetCalibration3D *curGroup = &groups[path];

	float limit = std::cos(fov/360*PI);

	// Convert to right-handed coordinate system, in a specific way to ensure repeatable import/export in Blender
	// For anything but Y Forward Axis and Up Axis Z import settings, blender applies a rotation
	// So pick conversion here specifically so these settings work nicely
	Eigen::Matrix3f convertAxis;
	convertAxis <<
	   -1, 0, 0,
		0, 1, 0,
		0, 0, 1;

	{
		while (true)
		{
			// Read line header if possible
			std::string header;
			if (!(fs >> header)) break;
			// Handle line of data
			if (header == "v")
			{ // Read vertex
				Eigen::Vector3f vert;
				fs >> vert.x();
				fs >> vert.y();
				fs >> vert.z();
				verts.push_back(vert);
			}
			else if (header == "vn")
			{ // Read normal
				Eigen::Vector3f nrm;
				fs >> nrm.x();
				fs >> nrm.y();
				fs >> nrm.z();
				nrms.push_back(nrm);
			}
			else if (header == "f")
			{ // Read face
				std::getline(fs, header);
				TargetMarker pt;
				pt.pos.setZero();
				pt.nrm.setZero();
				int pos = 0, sz = 0, count = 0;
				int vID, nID;
				while (sscanf(header.c_str() + pos, "%d//%d%n", &vID, &nID, &sz) == 2)
				{ // Sum vert values
					pos += sz;
					count++;
					pt.pos += verts.at(vID-1);
					pt.nrm += nrms.at(nID-1);
				}
				if (count == 0) return "";
				if (count < 3) continue; // Failed to read face, probably because UVs were exported
				// Calculate face center as average
				pt.pos = convertAxis * pt.pos/count;
				pt.nrm = convertAxis * pt.nrm.normalized();
				pt.viewAngle = limit;
				pt.size = size/1000.0f; // mm to m
				curGroup->markers.push_back(pt);
			}
			else if (header == "g")
			{ // Read next group name and assign current group
				std::getline(fs >> std::ws, header);
				curGroup = &groups[header];
			}
			else
			{ // Skip line
				std::getline(fs, header);
			}
		}

		// Turn vertex groups found into targets
		for (auto &group : groups)
		{
			if (group.first != "Base" && group.first != "(null)" && (groups.size() == 1 || group.first != path))
			{ // Given vertex group (polygroup) is a set of markers
				group.second.updateMarkers();
				targets.emplace(group.first, std::move(group.second));
			}
		}
	}

	return std::nullopt;
}

std::optional<ErrorMessage> writeTargetObjFile(const std::string &path, const std::string &label, const TargetCalibration3D &target)
{
	std::filesystem::path fsPath(path);
	std::filesystem::create_directories(fsPath.remove_filename());

	FILE *out = fopen(path.c_str(), "w");
	if (!out) return asprintf_s("Failed to open '%s' for writing!", path.c_str());

	auto writeVec = [out](std::string id, Eigen::Vector3f vec)
	{
		fprintf(out, "%s %f %f %f\n", id.c_str(), vec.x(), vec.y(), vec.z());
	};
	auto writeFace = [out](int idx, int len, int nrm)
	{
		fprintf(out, "f");
		for (int i = idx; i < idx+len; i++)
			fprintf(out, " %d//%d", i, nrm);
		fprintf(out, "\n");
	};

	fprintf(out, "g %s\ns off\n", label.c_str());

	// Convert to right-handed coordinate system, in a specific way to ensure repeatable import/export in Blender
	// For anything but Y Forward Axis and Up Axis Z import settings, blender applies a rotation
	// So pick conversion here specifically so these settings work nicely
	Eigen::Matrix3f convertAxis;
	convertAxis <<
	   -1, 0, 0,
		0, 1, 0,
		0, 0, 1;

	const int segments = 20;
	for (int i = 0; i < target.markers.size(); i++)
	{
		const TargetMarker &marker = target.markers[i];
		float s = marker.size/2; // Diameter to radius
		Eigen::Vector3f pos = convertAxis * marker.pos;
		Eigen::Vector3f nrm = convertAxis * marker.nrm;

		Eigen::Vector3f x = nrm.cross(Eigen::Vector3f::UnitX()).normalized();
		Eigen::Vector3f y = nrm.cross(x).normalized();
		assert(std::abs(x.norm()-1) < 0.001 && std::abs(y.norm()-1) < 0.001);
		writeVec("vn", nrm);
		for (int j = 0; j < segments; j++)
		{
			float r = 2*(float)PI * j/segments;
			writeVec("v", pos + s*x*std::sin(r) - s*y*std::cos(r));
		}
		writeFace(i*segments + 1, segments, i+1);
	}
	fclose(out);

	return std::nullopt;
}
