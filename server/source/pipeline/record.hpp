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

#ifndef RECORD_H
#define RECORD_H

#include "imu/imu.hpp"
#include "tracking/cluster.hpp"
#include "tracking/virtual.hpp"

#include "util/trackdef.hpp"
#include "util/eigendef.hpp"
#include "util/util.hpp" // TimePoint_t
#include "util/blocked_vector.hpp"

#include "ptr/value_ptr.hpp"

#include <vector>
#include <memory>

/**
 * Uncompressed image data of a camera frame
 */
struct CameraImage
{
	FrameID frameID;
	Bounds2i boundsPx;				// Coordinates in camera pixel space
	Bounds2f boundsRel;				// Coordinates are in (0,0,1,1) of source frame
	std::vector<uint8_t> image;
	int width, height;
	int jpegSize;
};

/**
 * Compressed image data of a camera frame
 */
struct CameraImageRecord
{
	CameraID cameraID;
	FrameID frameID;
	Bounds2i boundsPx;				// The area of the source frame covered by the image
	unsigned int frameX, frameY;	// The source frame size
	unsigned int imageX, imageY;	// The cropped and subsampled image size
	std::vector<uint8_t> jpeg;		// The image encoded as jpeg
	// Image Size != source frame size != boundsPx size!
	// Image is subsampled (integer multiple) of a cropped bounds of the source frame
};

struct BlobProperty
{
	float size; // (blob diameter in pixels) / (camera width in pixels), or radius in -1 to 1 space - max 255 pixels
	int value; // Metric derived from contrast and absolute brightness - max 1024

	// For MSVC...
	BlobProperty() {}
	BlobProperty(float size, int value) : size(size), value(value) {}
};

struct CameraFrameRecord
{
	bool received;
	std::vector<BlobProperty> properties; // Blob properties and metrics
	std::vector<Eigen::Vector2f> rawPoints2D; // Points2D input, normalised image coordinates
	std::vector<Eigen::Vector2f> points2D; // Points2D undistorted, normalised image coordinates

	// TODO: Consider switching to normalised camera coordinates for all systems
	// We could be working with points2D in normalised camera coordinates
	// Point calibration currently uses rawPoints2D (to apply camera projection inverse, including undistortion, with optimised parameters)
	// Everything else works converts normalised image coordinates to normalised camera coordinates on-demand with CameraCalib
	// So we COULD switch to normalised camera coordinates directly, and a lot of things would not NEED CameraCalib anymore
	// However, errors are always evaluated in normalised image coordinates to be consistent
	// This also allows easy representation as pixel errors, though they are representative only anyway (constant PixeLFactor), so wouldn't hurt to switch
	// This affects logging, but also optimisation (how, I don't know - maybe it's even better in camera coordinates?)
	// So it's a switch we COULD make, but it's a lot of effort, with minor benefits
	// Consider doing this whenever CameraMode is reworked (which will change CameraCalib to some degree, too)

	std::vector<Cluster2DStats> clusters2D;

	struct {
		std::vector<int> points2GTMarker;
		std::vector<int> GTMarkers2Point;
	} simulation = {};

	std::shared_ptr<CameraImageRecord> image; // Assigned only if pipeline.keepFrameImages is true
};

struct TargetMatchError
{
	float mean, stdDev, max;
	int samples;
};

struct TrackerVirtualError
{
	float pos, rot, posVar;
	int subtrackers;
};

using CovarianceMatrix = Eigen::Matrix<float,6,6>;

struct TargetCalibration3D;

struct TargetMatch2D
{
	std::vector<std::vector<std::pair<int,int>>> points2D;
	Eigen::Isometry3f pose;
	TargetMatchError error = {};
	int freeProjections, freeObservations;
	CovarianceMatrix covariance;
	std::vector<VectorX<float>> deviations; // Experimental: covariance from jacobian samples

	inline int count() const 
	{
		int count = 0;
		for (auto &camera : points2D)
			count += camera.size();
		return count;
	}
};

/**
 * Decimal value encoding tracking results including specific internal states
 */
class TrackingResult
{
public:
	enum Base
	{
		_BASE				= 0xFF << 8,
		IS_TEST				= 1 << 8,
		IS_DETECTED			= 2 << 8,
		IS_TRACKED			= 3 << 8,
		IS_FAILURE			= 4 << 8,
	};
	enum State
	{
		_STATE				= 0xFF,
		SEARCHED_2D			= IS_TEST | 1,
		MATCHED_3D			= IS_TEST | 2,
		PROBED_2D			= IS_TEST | 3,
		DETECTED_S2D		= IS_DETECTED | 1,
		DETECTED_M3D		= IS_DETECTED | 2,
		DETECTED_P2D		= IS_DETECTED | 3,
		INIT_VIRTUAL		= IS_DETECTED | 4,
		TRACKED_MARKER		= IS_TRACKED | 1,
		TRACKED_POSE		= IS_TRACKED | 2,
		TRACKED_SPARSE		= IS_TRACKED | 3,
		TRACKED_VIRTUAL		= IS_TRACKED | 4,
		NO_TRACK			= IS_FAILURE | 1,
	};
	enum Flag
	{
		_FLAG				= 0xFF << 16,
		NO_FLAG				= 0,
		FILTER_FAILED		= (1 << 16),
		CATCHING_UP			= (1 << 17),
		REMOVED				= (1 << 18),
		VIRT_MISTRUST		= (1 << 19),
	};
	enum Value : int
	{
		NONE = 0
	};

	TrackingResult(int value = TrackingResult::NONE) : value((Value)value) {}
	TrackingResult(TrackingResult::Base value) : value((Value)value) {}
	TrackingResult(TrackingResult::State state, TrackingResult::Flag flag = TrackingResult::NO_FLAG)
		: value((Value)((Value)state | (Value)flag)) {}

	constexpr operator TrackingResult::Value() const { return value; }
	constexpr operator TrackingResult::Value&() { return value; }

	bool operator==(TrackingResult::Value val) const { return value == val; }
	bool operator!=(TrackingResult::Value val) const { return value != val; }

	bool isProbe() const { return (value&_BASE) == IS_TEST; }
	bool isDetected() const { return (value&_BASE) == IS_DETECTED; }
	bool isTracked() const { return (value&_BASE) == IS_TRACKED; }
	bool isFailure() const { return (value&_BASE) == IS_FAILURE; }
	bool isState(TrackingResult::State state) const { return (value & (_BASE|_STATE)) == state; }
	TrackingResult::State getState() const { return (TrackingResult::State)(value & (_BASE|_STATE)); }
	bool hasFlag(TrackingResult::Flag flag) const { return (value & flag) == flag; }
	bool setFlag(TrackingResult::Flag flag){ return value = (Value)(value | (flag & _FLAG)); }

private:
	Value value;
};

enum class TrackerInertialState : uint8_t
{
	NO_IMU,
	IMU_CALIBRATING,
	IMU_TRACKING,
	IMU_LOST
};

struct TrackerPose
{
	// Pose as observed by the cameras
	Eigen::Isometry3f observed;
	// Pose filtered from all observations
	Eigen::Isometry3f filtered;
	// Covariances of those poses
	CovarianceMatrix observedCov;
	CovarianceMatrix filteredCov;
};

struct TrackerPoseExtended
{
	// Extrapolated using just the model from last observation
	Eigen::Isometry3f extrapolated;
	// Predicted using extrapolation and/or IMU integration
	Eigen::Isometry3f predicted;
	CovarianceMatrix predictedCov;
	// Only for trackers with IMUs:
	Eigen::Isometry3f inertialIntegrated; // Pose as integrated from new IMU samples
	Eigen::Isometry3f inertialFused; // Pose as integrated and fused with optical samples
	Eigen::Isometry3f inertialFiltered; // Pose as integrated and filtered with kalman filter
};

struct TrackerRecordVis
{
	// Target-specific
	std::vector<std::vector<int>> visibleMarkers;

	// Visual Tracker-specific
	std::vector<Eigen::Vector3f> virtualRelations; // Relations from virtual tracker to subtrackers in world space
	std::vector<VirtualSubtrackerDebug> virtualSubtrackers;
};

struct TrackerRecord
{ // 6-DOF tracker record
	int id;

	// Results
	TrackingResult result;
	float procTimeMS;
	float mistrust;
	TrackerPose pose;
	union
	{
		TargetMatchError error;
		TrackerVirtualError virtualError;
	};

	// Extended results for debug & visualisation (may be dropped)
	ptr::value_ptr<TrackerPoseExtended> ext;
	ptr::value_ptr<TrackerRecordVis> visual;
	ptr::value_ptr<TargetMatch2D> match2D;

	// IMU-specific
	TrackerInertialState imuState;
	float imuSampleInterval;
	TimePoint_t imuLastSample;

	TrackerRecord() {}
	TrackerRecord(int id, TrackingResult result, float procTimeMS)
		: id(id), result(result), procTimeMS(procTimeMS) {}
};

struct FrameRecord
{
	FrameID ID; // Might be externally provided and not continuous, and might conflict between SyncGroups
	FrameNum num; // Guaranteed to be continuous and equal the index in frame records - used only by pipeline for internal processing

	// Idea:
	// Sync group internal, continuous frame number generated by controller/rf-sync and shared with cameras for them to label their frames
	// If external frame ID exists, communicate those with host for each sync groups frame
	// Then, internally, all sync groups will announce their frames to coordinate allocation of global frame numbers
	// This will ensure frame numbers are monotonously increasing with time
	// And yes, sync group frames MAY be merged if time is close together and/or externally assigned frame ID is the same
	// FrameRecord will store just global frame number and frameID (which may be externally assigned or the sync group internal number)
	// - Involved sync groups should be apparent from cameras with received data - otherwise, would have to maintain a list for merged frames
	// - Similarly, separate sync group internal number may differ for two merged frames, so just not include it? Would something need it?


	TimePoint_t time; // Synced real time
	std::chrono::system_clock::time_point timeUTC;
	bool finishedProcessing;

	// Camera input data
	std::vector<CameraFrameRecord> cameras; // indexed like pipeline.cameras

	// Tracking results
	// Async Detection writes to frame->trackers while UI is reading, meaning it has to be thread-safe (1/5)
	// There are two ways to make this thread-safe without synchronisation primitives (since this is data storage):
	// - std::list, since iterators remain valid even as the list gets appended to
	// - std::vector preallocated to cover any additional entries (currently limited to 1 due to async detection)
	// Since this is data storage, std::vector provides a better memory layout
	std::vector<TrackerRecord> trackers;

	// TODO: Track individual large markers (4/4)
	// Either store in tracker record or here in separate records

	// TODO: Properly integrate triangulation records (1/3)
	std::vector<Eigen::Vector3f> triangulations;

	// This is mostly for visualisation
	std::vector<Cluster3D> cluster2DTri;

	// Points not used for tracking during realtime processing
	// Useful for retroactive tracking e.g. after a detection
	std::vector<std::vector<int>> remainingPoints2D;
};

/**
 * Record of all data for replay as well as tracking results
 */
struct TrackingRecord
{
	BlockedQueue<std::shared_ptr<FrameRecord>> frames;
	std::vector<std::shared_ptr<IMU>> imus;
};

/**
 * Record of one trackers data used for comparison
 */
struct TrackerCompareRecord
{
	int trackerID;
	std::string label;
	int type;
	BlockedQueue<std::shared_ptr<FrameRecord>> frames;
};

#endif // RECORD_H