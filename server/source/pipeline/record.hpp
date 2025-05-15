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

struct TrackedTargetRecord
{
	int id;
	bool tracked;
	TargetMatchError error;
	Eigen::Isometry3f posePredicted, poseExtrapolated, poseObserved, poseFiltered;
	Eigen::Isometry3f poseInertialIntegrated, poseInertialFused, poseInertialFiltered;
	CovarianceMatrix covPredicted, covObserved, covFiltered;

	// For visualisation only (including predicted)
	// TODO: Consider garbage-collecting intermediate results from frameRecords
	// Storing this for all frames, forever, is not great
	// However, we may visualise a past frame, usually just because the current frame is different and still processing
	// Maybe retroactively garbage-collect frame record if neither visualisation, async detection, etc. needs it
	std::vector<std::vector<int>> visibleMarkers;
};

struct FrameRecord
{
	FrameID ID; // Might be externally provided and not continuous, and might conflict between SyncGroups
	unsigned int num; // Guaranteed to be continuous (in what?) - used only by pipeline for internal processing

	// TODO: How to handle two synced groups of cameras, what is a "frame" there?
	// Use expected sync time to pre-assign num so that missing frames are apparent?
	// What if expected time of SyncGroups is so close together that a distinct num is not reliable?
	// -> merge SyncGroups for those cases and wait for all cameras? Could yield better results anyway

	// FrameID varies between sync groups, each controller keeps track of it's own FrameID for the cameras it manages
	// However, in the future, external sync protocols might introduce non-predictable frameIDs, though they probably should always be monotonic
	// Maybe controllers will have a separate syncID that is predictable, and frameRecords use that, allowing us to find the frameRecords faster
	// Then frameID can be anything a potential external sync protocol wants it to be

	// Needs adjustment in server::ProcessStreamFrame too, where num is currently enforced to be equal to ID

	TimePoint_t time; // Synced real time
	bool finishedProcessing;

	// Camera input data
	std::vector<CameraFrameRecord> cameras; // indexed like pipeline.cameras

	// Tracking results
	struct 
	{
		int trackingLosses, searches2D, detections2D, detections3D, trackingCatchups;
		std::vector<TrackedTargetRecord> targets;
		//std::vector<TrackedMarkerRecord> markers;
		std::vector<Eigen::Vector3f> triangulations;
	} tracking;

	// TODO: Put remainingPoints2D and other intermediate results in here?
	// Both target calibration (tryTrackFrame) and async detection2D (retroactivelyTrackFrame) would benefit
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