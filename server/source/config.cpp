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

#include "pipeline/frameRecord.hpp"
#include "target/target.hpp"
#include "point/sequence_data.hpp"
#include "calib_target/assembly.hpp"

#include "util/eigenutil.hpp"
#include "util/blocked_vector.hpp"
#include "util/log.hpp"

#include "nlohmann/json.hpp"
using json = nlohmann::json;

#include <set>
#include <fstream>
#include <iomanip>
#include <filesystem>

/**
 * Parsing and writing of Config and other files
 */


int findLastFileEnumeration(const char* pathFormat)
{
	int i = -1;
	std::string obsPath;
	do obsPath = asprintf_s(pathFormat, ++i);
	while (std::filesystem::exists(std::filesystem::path(obsPath)));
	return i-1;
}

void parseGeneralConfigFile(const std::string &path, GeneralConfig &config)
{
	// Read JSON config file
	std::ifstream fs(path);
	if (!fs.is_open()) return;
	json cfg;
	fs >> cfg;

	// Parse JSON config file

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
			if (targets.is_array())
			{
				for (auto &md : targets)
				{
					if (md.is_string())
						parseTargetObjFile(md, config.simulation.trackingTargets, defaultMarkerFoV, defaultMarkerSize);
					else if (md.is_object())
					{
						float v = md.contains("markerViewAngle") && md["markerViewAngle"].is_number()? md["markerViewAngle"].get<float>() : defaultMarkerFoV;
						float s = md.contains("markerSizeMM") && md["markerSizeMM"].is_number()? md["markerSizeMM"].get<float>() : defaultMarkerSize;
						parseTargetObjFile(md["path"], config.simulation.trackingTargets, v, s);
					}
				}
			}
			else if (targets.is_string())
				parseTargetObjFile(targets, config.simulation.trackingTargets, defaultMarkerFoV, defaultMarkerSize);
		}
		if (simulation.contains("cameras"))
		{
			auto &cameras = simulation["cameras"];
			if (cameras.is_string())
			{
				config.simulation.cameraDefPath = cameras.get<std::string>();
				parseCameraCalibrations(cameras.get<std::string>(), config.simulation.cameraDefinitions);
			}
		}
	}
}

void storeGeneralConfigFile(const std::string &path, const GeneralConfig &config)
{
	// TODO
}

void parseCameraConfigFile(const std::string &path, CameraConfigMap &configMap)
{
	// Read JSON config file
	std::ifstream fs(path);
	if (!fs.is_open()) return;
	json calib;
	fs >> calib;
	fs.close();

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
				int id = cam["id"].get<int>();
				int cfg = cam["configuration"].get<int>();
				if (cfg >= configMap.configurations.size()) continue;
				configMap.cameraConfigs[id] = cfg;
				LOGC(LInfo, "Loaded config %d for camera %d!", cfg, id);
			}
		}
	}
}

void storeCameraConfigFile(const std::string &path, const CameraConfigMap &config)
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

	// Write JSON calib file
	std::filesystem::create_directories(std::filesystem::path(path).remove_filename());
	std::ofstream fs(path);
	if (!fs.is_open()) return;
	fs << std::setw(4) << file;
	fs.close();
}

void parseCameraCalibrations(const std::string &path, std::vector<CameraCalib> &cameraCalib)
{
	// Read JSON config file
	std::ifstream fs(path);
	if (!fs.is_open()) return;
	json calib;
	fs >> calib;
	fs.close();

	// Parse camera calibration
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
					calib.id = jsCamera["id"].get<int>();
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
				// Parse principal point
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

void storeCameraCalibrations(const std::string &path, const std::vector<CameraCalib> &cameraCalib)
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

	// Write JSON calib file
	std::filesystem::create_directories(std::filesystem::path(path).remove_filename());
	std::ofstream fs(path);
	if (!fs.is_open()) return;
	fs << std::setw(4) << file;
	fs.close();
}

void parseTargetCalibrations(const std::string &path, std::vector<TargetTemplate3D> &targetTemplates)
{
	// Read JSON config file
	std::ifstream fs(path);
	if (!fs.is_open()) return;
	json file;
	fs >> file;
	fs.close();

	// Parse target calibration
	if (file.contains("targets"))
	{
		auto &jsTargets = file["targets"];
		if (jsTargets.is_array())
		{
			targetTemplates.reserve(targetTemplates.size() + jsTargets.size());
			for (auto &jsTarget : jsTargets)
			{
				TargetTemplate3D target;
				if (!jsTarget.contains("id")) continue;
				target.id = jsTarget["id"].get<int>();
				target.label = jsTarget["label"].get<std::string>();

				float defaultAngle = jsTarget.contains("DefaultAngle")? jsTarget["DefaultAngle"].get<float>() : 180;
				float defaultSize = (jsTarget.contains("DefaultSize")? jsTarget["DefaultSize"].get<float>() : 8) / 1000.0f;
				float defaultLimit = std::cos(defaultAngle/360*PI);

				if (!jsTarget.contains("markers")) continue;
				auto &jsMarkers = jsTarget["markers"];
				if (!jsMarkers.is_array()) continue;
				target.markers.reserve(jsMarkers.size());
				for (auto &jsMarker : jsMarkers)
				{
					TargetMarker marker = {};
					if (!jsMarker.is_object()) continue;
					// Read position
					if (!jsMarker.contains("Position")) continue;
					auto &pos = jsMarker["Position"];
					if (!pos.is_array() || pos.size() < 3) continue;
					marker.pos = Eigen::Vector3f(pos[0].get<float>(), pos[1].get<float>(), pos[2].get<float>());
					// Read normals
					if (!jsMarker.contains("Normal")) continue;
					auto &nrm = jsMarker["Normal"];
					if (!nrm.is_array() || nrm.size() < 3) continue;
					marker.nrm = Eigen::Vector3f(nrm[0].get<float>(), nrm[1].get<float>(), nrm[2].get<float>());
					// Read other values
					if (jsMarker.contains("Angle")) // In degrees
						marker.angleLimit = std::cos(jsMarker["Angle"].get<float>()/360*PI);
					else
						marker.angleLimit = defaultLimit;
					if (jsMarker.contains("Size")) // In mm
						marker.size = jsMarker["Size"].get<float>() / 1000.0f;
					else
						marker.size = defaultSize;
					target.markers.push_back(std::move(marker));
				}
				target.updateMarkers();
				targetTemplates.push_back(std::move(target));
			}
		}
	}
}

void storeTargetCalibrations(const std::string &path, const std::vector<TargetTemplate3D> &targetTemplates)
{
	json file;

	// Write target calibration
	file["targets"] = json::array();
	for (auto &target : targetTemplates)
	{
		json jsTarget;
		jsTarget["id"] = target.id;
		jsTarget["label"] = target.label;
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
			jsMarker["Angle"] = std::acos(marker.angleLimit)*360/PI;
			jsMarker["Size"] = marker.size * 1000.0f;
			jsMarkers.push_back(std::move(jsMarker));
		}
		file["targets"].push_back(std::move(jsTarget));
	}

	// Write JSON calib file
	std::filesystem::create_directories(std::filesystem::path(path).remove_filename());
	std::ofstream fs(path);
	if (!fs.is_open()) return;
	fs << std::setw(4) << file;
	fs.close();
}

unsigned int parseFrameRecords(const std::string &path, std::vector<CameraConfigRecord> &cameras, std::vector<FrameRecord> &frameRecords)
{
	std::filesystem::path imgFolder = std::filesystem::path(path).replace_extension();

	// Read JSON file
	std::ifstream fs(path);
	if (!fs.is_open()) return 0;
	json file;
	fs >> file;
	fs.close();

	unsigned int frameOffset;
	try {
		if (!file.contains("frameRecords")) return 0;
		if (!file["frameRecords"].is_object()) return 0;
		auto &jsRecords = file["frameRecords"];

		if (!jsRecords.contains("frames")) return 0;
		if (!jsRecords["frames"].is_array()) return 0;
		auto &jsFrames = jsRecords["frames"];

		if (jsRecords.contains("cameras") && jsRecords["cameras"].is_array())
		{
			auto &jsCameras = jsRecords["cameras"];
			cameras.reserve(jsCameras.size());
			for (auto &jsCamera : jsCameras)
			{
				if (jsCamera.is_object())
				{
					if (!jsCamera.contains("id")) return 0;
					cameras.emplace_back(jsCamera["id"].get<CameraID>(),
						jsCamera.contains("frameX")? jsCamera["frameX"].get<int>() : 1280,
						jsCamera.contains("frameY")? jsCamera["frameY"].get<int>() : 800
					);
				}
				else if (jsCamera.is_number_integer())
					cameras.emplace_back(jsCamera.get<CameraID>(), 1280, 800);
				else
					return 0;
			}
		}
		else if (jsRecords.contains("cameraCount") && jsRecords["cameraCount"].is_number_integer())
		{
			cameras.reserve(jsRecords["cameraCount"].get<int>());
			for (int i = 0; i < jsRecords["cameraCount"].get<int>(); i++)
				cameras.emplace_back(i+1, 1280, 800);
		}
		else
			return 0;

		frameRecords.clear();
		frameRecords.reserve((std::size_t)(jsFrames.size()*1.01f+100));
		TimePoint_t startT = sclock::now();
		bool foundFirst = false;
		for (auto &jsFrame : jsFrames)
		{
			if (!jsFrame.contains("cameras")) continue;
			if (!jsFrame["cameras"].is_array()) continue;
			auto &jsCameras = jsFrame["cameras"];

			unsigned int num = jsFrame["num"].get<unsigned int>();
			if (!foundFirst)
			{
				frameOffset = num;
				foundFirst = true;
			}
			frameRecords.resize(num-frameOffset+1);
			FrameRecord &frame = frameRecords[num-frameOffset];
			frame.ID = jsFrame["id"].get<unsigned int>();
			frame.num = num-frameOffset;
			frame.time = startT + std::chrono::microseconds(jsFrame["dt"].get<unsigned long>());
			frame.cameras.reserve(cameras.size());

			for (auto &jsCamera : jsCameras)
			{
				if (frame.cameras.size() >= cameras.size())
					return frameOffset;
				if (!jsCamera.contains("blobs")) continue;
				if (!jsCamera["blobs"].is_array()) continue;
				auto &jsBlobs = jsCamera["blobs"];

				frame.cameras.push_back({});
				auto &camera = frame.cameras.back();
				camera.rawPoints2D.reserve(jsBlobs.size());
				camera.properties.reserve(jsBlobs.size());

				for (auto &jsBlob : jsBlobs)
				{
					camera.rawPoints2D.emplace_back(jsBlob["x"].get<float>(), jsBlob["y"].get<float>());
					camera.properties.emplace_back(
						jsBlob["s"].get<float>(),
						jsBlob.contains("v")? jsBlob["v"].get<float>() : 1000);
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
				int camIndex = frame.cameras.size()-1;
				image.cameraID = cameras[camIndex].ID;
				image.frameID = frame.ID;
				image.jpeg = std::move(jpeg);

				image.frameX = jsImage.contains("frameX")? jsImage["frameX"].get<int>() : cameras[camIndex].frameX;
				image.frameY = jsImage.contains("frameY")? jsImage["frameY"].get<int>() : cameras[camIndex].frameY;

				if (jsImage.contains("min") && jsImage.contains("max"))
				{
					image.boundsPx.minX = jsImage["min"]["x"].get<int>();
					image.boundsPx.minY = jsImage["min"]["y"].get<int>();
					image.boundsPx.maxX = jsImage["max"]["x"].get<int>();
					image.boundsPx.maxY = jsImage["max"]["y"].get<int>();
				}
				else
					image.boundsPx = Bounds2i(0, 0, image.frameX, image.frameY);

				image.imageX = jsImage.contains("imageX")? jsImage["imageX"].get<int>() : image.boundsPx.extends().x();
				image.imageY = jsImage.contains("imageY")? jsImage["imageY"].get<int>() : image.boundsPx.extends().y();
			}
		}
	}
	catch(json::exception e)
	{
		LOG(LDefault, LWarn, "Failed to fully parse JSON file %s!", path.c_str())
		return frameOffset;
	}

	return frameOffset;
}

template<typename Iterator>
void dumpFrameRecords(const std::string &path, const Iterator &frameStart, const Iterator &frameEnd, const std::vector<CameraConfigRecord> &cameras)
{
	if (frameStart == frameEnd)
		return;
	// Open file first
	std::filesystem::create_directories(std::filesystem::path(path).remove_filename());
	std::filesystem::path imgFolder = std::filesystem::path(path).replace_extension();
	std::ofstream fs(path);
	if (!fs.is_open()) return;
	fs << std::flush;

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

	// Write observations
	file["frameRecords"]["frames"] = json::array();
	auto &jsFrames = file["frameRecords"]["frames"];
	TimePoint_t start = sclock::now();
	bool foundFirst = false;
	for (Iterator frameIt = frameStart; frameIt != frameEnd; frameIt++)
	{
		if (!frameIt->get()) continue; // No frame recorded, can happen due to record delay (first frames), or dropped frames from hardware
		const FrameRecord &frame = *frameIt->get();
		if (!foundFirst)
		{
			start = frame.time;
			foundFirst = true;
		}
		json jsFrame;
		jsFrame["id"] = frame.ID;
		jsFrame["num"] = frame.num;
		jsFrame["dt"] = dtUS(start, frame.time);
		jsFrame["cameras"] = json::array();
		int camIndex = 0;
		for (auto &camera : frame.cameras)
		{
			json jsCamera;
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
						jsImage["min"]["x"] = image.boundsPx.minX;
						jsImage["min"]["y"] = image.boundsPx.minY;
						jsImage["max"]["x"] = image.boundsPx.maxX;
						jsImage["max"]["y"] = image.boundsPx.maxY;
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

	// Write JSON calib file
	fs << std::setw(4) << file;
	fs.close();
}

template void dumpFrameRecords(const std::string &path, const BlockedQueue<std::shared_ptr<FrameRecord>>::const_iterator &frameStart, const BlockedQueue<std::shared_ptr<FrameRecord>>::const_iterator &frameEnd, const std::vector<CameraConfigRecord> &cameras);

void parseTrackingResults(std::string path, std::vector<FrameRecord> &frameRecords, unsigned int frameOffset)
{
	// Modify path (expecting frame_capture_XX.json)
	std::size_t index = path.rfind("/frame_capture_");
	if (index != std::string::npos)
		path.replace(index, 15, "/frame_tracking_");

	// Read JSON file
	std::ifstream fs(path);
	if (!fs.is_open()) return;
	json file;
	fs >> file;
	fs.close();

	try {
		if (!file.contains("trackingResults")) return;
		if (!file["trackingResults"].is_object()) return;
		auto &jsRecords = file["trackingResults"];

		if (!jsRecords.contains("frames")) return;
		if (!jsRecords["frames"].is_array()) return;
		auto &jsFrames = jsRecords["frames"];

		for (auto &jsFrame : jsFrames)
		{
			if (!jsFrame.contains("num")) continue;
			if (!jsFrame.contains("targets")) continue;
			if (!jsFrame["targets"].is_array()) continue;
			auto &jsTargets = jsFrame["targets"];

			unsigned int num = jsFrame["num"].get<unsigned int>();
			if (num < frameOffset) continue;
			if (num > frameOffset+frameRecords.size()) break;

			FrameRecord &frame = frameRecords[num-frameOffset];
			frame.tracking = {};
			frame.tracking.targets.reserve(jsTargets.size());
			for (auto &jsTarget : jsTargets)
			{
				if (!jsTarget.is_object() || !jsTarget.contains("pose") || !jsTarget["pose"].is_array() || jsTarget["pose"].size() != 4*4) continue;
				frame.tracking.targets.push_back({});
				auto &target = frame.tracking.targets.back();
				target.id = jsTarget["id"].get<int>();
				int i = 0;
				auto poseArr = target.poseObserved.matrix().array();
				for (auto &val : jsTarget["pose"])
					poseArr(i++) = val.is_number_float()? val.get<float>() : NAN;
				target.error.samples = jsTarget["num"].get<int>();
				target.error.mean = jsTarget["avg"].get<float>();
				target.error.stdDev = jsTarget.contains("dev")? jsTarget["dev"].get<float>() : 0.0f;
				target.error.max = jsTarget["max"].get<float>();
			}

			if (!jsFrame.contains("events")) continue;
			if (!jsFrame["events"].is_array()) continue;
			for (auto &evt : jsFrame["events"])
			{
				switch (evt.get<int>())
				{
					case 1:
						frame.tracking.trackingLosses++;
						break;
					case 2:
						frame.tracking.searches2D++;
						break;
					case 3:
						frame.tracking.detections2D++;
						break;
					case 4:
						frame.tracking.detections3D++;
						break;
					case 5:
						frame.tracking.trackingCatchups++;
						break;
					default:
						break;
				}
			}
		}
	}
	catch(json::exception e)
	{
		LOG(LDefault, LWarn, "Failed to fully parse JSON file %s!", path.c_str())
		return;
	}
}

template<typename Iterator>
void dumpTrackingResults(std::string path, const Iterator &frameStart, const Iterator &frameEnd, unsigned int frameOffset)
{
	if (frameStart == frameEnd)
		return;
	// Modify path (expecting frame_capture_XX.json)
	std::size_t index = path.rfind("/frame_capture_");
	if (index != std::string::npos)
		path.replace(index, 15, "/frame_tracking_");
	// Open file first
	std::ofstream fs(path);
	if (!fs.is_open()) return;
	fs << std::flush;

	json file;

	std::set<int> targetIDs;

	// Write observations
	file["trackingResults"] = json::object();
	auto &jsRecords = file["trackingResults"];
	jsRecords["frames"] = json::array();
	auto &jsFrames = jsRecords["frames"];
	for (Iterator frameIt = frameStart; frameIt != frameEnd; frameIt++)
	{
		if (!frameIt->get()) continue; // No frame recorded, can happen due to record delay (first frames), or dropped frames from hardware
		const FrameRecord &frame = *frameIt->get();
		int eventCnt = frame.tracking.trackingLosses + frame.tracking.searches2D + frame.tracking.detections2D + frame.tracking.detections3D + frame.tracking.trackingCatchups;
		if (frame.tracking.targets.empty() && eventCnt == 0)
			continue;
		json jsFrame;
		jsFrame["id"] = frame.ID;
		jsFrame["num"] = frameOffset + frame.num;
		jsFrame["targets"] = json::array();
		for (const auto &target : frame.tracking.targets)
		{
			json jsTarget;
			jsTarget["id"] = target.id;
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
		if (eventCnt > 0)
		{
			jsFrame["events"] = json::array();
			for (int i = 0; i < frame.tracking.trackingLosses; i++)
				jsFrame["events"].push_back(1);
			for (int i = 0; i < frame.tracking.searches2D; i++)
				jsFrame["events"].push_back(2);
			for (int i = 0; i < frame.tracking.detections2D; i++)
				jsFrame["events"].push_back(3);
			for (int i = 0; i < frame.tracking.detections3D; i++)
				jsFrame["events"].push_back(4);
			for (int i = 0; i < frame.tracking.trackingCatchups; i++)
				jsFrame["events"].push_back(5);
		}
		jsFrames.push_back(std::move(jsFrame));
	}

	// Write trackers tracked during frame span
	jsRecords["trackers"] = json::array();
	for (int tgtID : targetIDs)
		jsRecords["trackers"].push_back(tgtID);

	// Write JSON calib file
	fs << std::setw(4) << file;
	fs.close();
}

template void dumpTrackingResults(std::string path, const BlockedQueue<std::shared_ptr<FrameRecord>>::const_iterator &frameStart, const BlockedQueue<std::shared_ptr<FrameRecord>>::const_iterator &frameEnd, unsigned int frameOffset);


SequenceData parseSequenceDatabase(const std::string &path, std::vector<CameraID> &cameraIDs)
{
	SequenceData sequences = {};

	// Read JSON file
	std::ifstream fs(path);
	if (!fs.is_open()) return sequences;
	json file;
	fs >> file;
	fs.close();

	if (!file.contains("observations")) return sequences;
	if (!file["observations"].is_object()) return sequences;
	auto &jsObs = file["observations"];

	if (!jsObs.contains("markers")) return sequences;
	if (!jsObs["markers"].is_array()) return sequences;
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
		return sequences;
	sequences.temporary.resize(cameraCount);

	sequences.markers.resize(jsMarkers.size());
	int m = 0;
	for (auto &jsMarker : jsMarkers)
	{
		if (!jsMarker.contains("cameras")) continue;
		if (!jsMarker["cameras"].is_array()) continue;
		auto &jsCameras = jsMarker["cameras"];

		MarkerSequences &marker = sequences.markers[m++];
		marker.cameras.resize(cameraCount);

		int c = 0;
		for (auto &jsCamera : jsCameras)
		{
			if (!jsCamera.contains("sequences")) continue;
			if (!jsCamera["sequences"].is_array()) continue;
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

		sequences.lastRecordedFrame = std::max(sequences.lastRecordedFrame, marker.lastFrame);
	}

	return sequences;
}

void dumpSequenceDatabase(const std::string &path, const SequenceData &sequences, const std::vector<CameraID> &cameraIDs)
{
	if (sequences.markers.size() == 0)
		return;
	// Open file first
	std::filesystem::create_directories(std::filesystem::path(path).remove_filename());
	std::ofstream fs(path);
	if (!fs.is_open()) return;
	fs << std::flush;

	json file;

	// Write camera map
	file["observations"]["cameras"] = json::array();
	auto &cameraMap = file["observations"]["cameras"];
	for (int id : cameraIDs)
		cameraMap.push_back(id);

	// Write observations
	file["observations"]["markers"] = json::array();
	auto &jsMarkers = file["observations"]["markers"];
	for (auto &marker : sequences.markers)
	{
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

	// Write JSON calib file
	fs << std::setw(4) << file;
	fs.close();
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

std::vector<std::shared_ptr<TargetView>> parseTargetViewRecords(const std::string &path,
	const BlockedQueue<std::shared_ptr<FrameRecord>> &frameRecords)
{
	std::vector<std::shared_ptr<TargetView>> views;

	// Read JSON file
	std::ifstream fs(path);
	if (!fs.is_open()) return views;
	json file;
	fs >> file;
	fs.close();

	if (!file.contains("views")) return views;
	if (!file["views"].is_array()) return views;
	auto &viewArr = file["views"];

	views.reserve(viewArr.size());
	std::size_t recordCount = frameRecords.getView().size();
	for (auto &view : viewArr)
	{
		ObsTarget target;
		if (!parseOptTarget(view, target)) continue;

		std::shared_ptr<TargetView> tgtView = std::make_shared<TargetView>();
		tgtView->state.calibrated = true;
		tgtView->selected = true;
		tgtView->id = views.size();
		tgtView->beginFrame = target.frames.front().frame;
		tgtView->endFrame = target.frames.back().frame;
		if (tgtView->endFrame >= recordCount) continue;
		tgtView->targetTemplate.initialise(target.markers);
		*tgtView->target.contextualLock() = std::move(target);
		views.push_back(std::move(tgtView));
	}

	return views;
}

void dumpTargetViewRecords(const std::string &path, const std::vector<std::shared_ptr<TargetView>> &views)
{
	// Open file first
	std::filesystem::create_directories(std::filesystem::path(path).remove_filename());
	std::ofstream fs(path);
	if (!fs.is_open()) return;
	fs << std::flush;

	json file;

	// Write views
	file["views"] = json::array();

	for (const auto &tgtView : views)
	{
		if (!tgtView->state.calibrated || !tgtView->selected) continue;
		
		json view;
		storeOptTarget(view, *tgtView->target.contextualRLock());
		file["views"].push_back(std::move(view));
	}

	// Write JSON calib file
	fs << std::setw(4) << file;
	fs.close();
}

bool parseTargetAssemblyStage(const std::string &path, TargetAssemblyBase &base)
{
	// Read JSON file
	std::ifstream fs(path);
	if (!fs.is_open()) return false;
	json file;
	fs >> file;
	fs.close();

	if (!file.contains("initialViewID")) return false;
	if (!file["initialViewID"].is_number_integer()) return false;
	base.initialViewID = file["initialViewID"].get<int>();

	if (!file.contains("merged")) return false;
	if (!file["merged"].is_array()) return false;
	base.merged.reserve(file["merged"].size());
	for (auto &m : file["merged"])
		base.merged.push_back(m.get<int>());

	if (!file.contains("target")) return false;
	if (!file["target"].is_object()) return false;
	return parseOptTarget(file["target"], base.target);
}

void dumpTargetAssemblyStage(const std::string &path, const TargetAssemblyBase &base)
{
	// Open file first
	std::filesystem::create_directories(std::filesystem::path(path).remove_filename());
	std::ofstream fs(path);
	if (!fs.is_open()) return;
	fs << std::flush;

	json file;
	file["initialViewID"] = base.initialViewID;

	// Write merged target list
	file["merged"] = json::array();
	for (int merged : base.merged)
		file["merged"].push_back(merged);

	// Write target with markers, sequence mapping, frames and their poses
	storeOptTarget(file["target"], base.target);

	// Write JSON calib file
	fs << std::setw(4) << file;
	fs.close();
}

bool parseTargetObjFile(const std::string &path, std::vector<TargetTemplate3D> &targets, float fov, float size)
{
	std::vector<Eigen::Vector3f> verts;
	std::vector<Eigen::Vector3f> nrms;

	std::ifstream fs(path);
	if (!fs.is_open()) return false;

	std::map<std::string, TargetTemplate3D> groups = { { path, TargetTemplate3D() } };
	TargetTemplate3D *curGroup = &groups[path];

	float limit = std::cos(fov/360*PI);

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
			fs >> vert.z();
			fs >> vert.y();
			verts.push_back(vert);
		}
		else if (header == "vn")
		{ // Read normal
			Eigen::Vector3f nrm;
			fs >> nrm.x();
			fs >> nrm.z();
			fs >> nrm.y();
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
			while(sscanf(header.c_str() + pos, "%d//%d%n", &vID, &nID, &sz) == 2)
			{ // Sum vert values
				pos += sz;
				count++;
				pt.pos += verts.at(vID-1);
				pt.nrm += nrms.at(nID-1);
			}
			if (count == 0) return false;
			if (count < 3) continue; // Failed to read face, probably because UVs were exported
			// Calculate face center as average
			pt.pos = pt.pos/count;
			pt.nrm.normalize();
			pt.angleLimit = limit;
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
			group.second.id = -(targets.size() + 1);
			group.second.label = std::move(group.first);
			group.second.updateMarkers();
			targets.push_back(std::move(group.second));
		}
	}
	fs.close();
	return true;
}

void writeTargetObjFile(const std::string &path, const TargetTemplate3D &targetTemplate)
{
	char filename[1000];
	sprintf(filename, path.c_str(), targetTemplate.id); // path is from internal code only, so fine to use as format string
	std::filesystem::create_directories(std::filesystem::path(path).remove_filename());
	FILE *out = fopen(filename, "w");

	auto writeVec = [out](std::string id, Eigen::Vector3f vec)
	{
		fprintf(out, "%s %f %f %f\n", id.c_str(), vec.x(), vec.y(), vec.z());
	};
	auto writeQuad = [out](int i)
	{
		fprintf(out, "f %d//%d %d//%d %d//%d\n", i+0, i+0, i+1, i+1, i+2, i+2);
		fprintf(out, "f %d//%d %d//%d %d//%d\n", i+1, i+1, i+2, i+2, i+3, i+3);
	};

	fprintf(out, "g Target_%d\ns off\n", targetTemplate.id);

	float s = 2.0f/1000;
	for (int i = 0; i < targetTemplate.markers.size(); i++)
	{
		const TargetMarker &marker = targetTemplate.markers[i];
		Eigen::Vector3f x = marker.nrm.cross(Eigen::Vector3f::UnitX());
		Eigen::Vector3f y = marker.nrm.cross(Eigen::Vector3f::UnitY());
		writeVec("v", marker.pos + s*x + s*y);
		writeVec("v", marker.pos - s*x + s*y);
		writeVec("v", marker.pos + s*x - s*y);
		writeVec("v", marker.pos - s*x - s*y);
		writeVec("vn", marker.nrm);
		writeVec("vn", marker.nrm);
		writeVec("vn", marker.nrm);
		writeVec("vn", marker.nrm);
		writeQuad(i*4 + 1);
	}
	fclose(out);
}
