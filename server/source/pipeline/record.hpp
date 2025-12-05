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

#include "util/eigendef.hpp"
#include "util/util.hpp" // TimePoint_t
#include "util/blocked_vector.hpp"

#include <vector>
#include <memory>

typedef uint32_t FrameID;

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
	float size;
	int value;
};

struct CameraFrameRecord
{
	bool received;
	std::vector<BlobProperty> properties; // Point sizes input, normalised (size in pixels / camera width in pixels)
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

using CovarianceMatrix = Eigen::Matrix<float,6,6>;

struct TargetCalibration3D;

struct TargetMatch2D
{
	std::vector<std::vector<std::pair<int,int>>> points2D;
	Eigen::Isometry3f pose;
	TargetMatchError error = {};
	CovarianceMatrix covariance;
	std::vector<VectorX<float>> deviations;

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
		TRACKED_MARKER		= IS_TRACKED | 1,
		TRACKED_POSE		= IS_TRACKED | 2,
		TRACKED_SPARSE		= IS_TRACKED | 3,
		NO_TRACK			= IS_FAILURE | 1,
	};
	enum Flag
	{
		_FLAG				= 0xFF << 16,
		NO_FLAG				= 0,
		FILTER_FAILED		= (1 << 16),
		CATCHING_UP			= (1 << 17),
		REMOVED				= (1 << 18),
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

enum class TrackerInertialState 
{
	NO_IMU,
	IMU_CALIBRATING,
	IMU_TRACKING,
	IMU_LOST
};

struct TrackerRecord
{
	int id;

	TrackingResult result;
	float procTimeMS;
	TargetMatchError error;
	float mistrust;
	int freeProjections, freeObservations;

	TrackerInertialState imuState;
	float imuSampleInterval;
	TimePoint_t imuLastSample;

	Eigen::Isometry3f poseObserved, poseFiltered;
	CovarianceMatrix covObserved, covFiltered;

	// Following is mostly for debug
	// TODO: Consider reducing memory-footprint of frameRecords in release builds
	// Extended filter output for visualisation (e.g. in trail)
	Eigen::Isometry3f posePredicted, poseExtrapolated;
	Eigen::Isometry3f poseInertialIntegrated, poseInertialFused, poseInertialFiltered;
	CovarianceMatrix covPredicted;
	// Intermediary tracking results for simulating filter again after modifying parameters
	TargetMatch2D match2D;

	// Following is for more general visualisation beyond debugging
	// TODO: Consider garbage-collecting visualisation data in frameRecords or storing it separately
	std::vector<std::vector<int>> visibleMarkers;
};

struct FrameRecord
{
	FrameID ID; // Might be externally provided and not continuous, and might conflict between SyncGroups
	unsigned int num; // Guaranteed to be continuous (in what?) - used only by pipeline for internal processing

	// TODO: How to handle two separately synced groups of cameras, what is a "frame" there?
	// Use expected sync time to pre-assign num so that missing frames are apparent?
	// What if expected time of SyncGroups is so close together that a distinct num is not reliable?
	// -> merge SyncGroups for those cases and wait for all cameras? Could yield better results anyway

	// FrameID varies between sync groups, each controller keeps track of it's own FrameID for the cameras it manages
	// However, in the future, external sync protocols might introduce non-predictable frameIDs, though they probably should always be monotonic
	// Maybe controllers will have a separate syncID that is predictable, and frameRecords use that, allowing us to find the frameRecords faster
	// Then frameID can be anything a potential external sync protocol wants it to be

	TimePoint_t time; // Synced real time
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

#endif // RECORD_H