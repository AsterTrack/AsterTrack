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

#ifndef TARGET_TRACKING_2D_H
#define TARGET_TRACKING_2D_H

#include "target/target.hpp"
#include "target/parameters.hpp"

#include "pipeline/record.hpp"

#include "util/eigendef.hpp"

/**
 * Fitting a target (set of markers) to observed points in 2D space
 */


/* Structures */

// Expose internal data for visualisation purposes (low overhead)
struct UncertaintyAxisAlignment
{
	Eigen::Vector2f rayCam, rayTgt, rayDir;
	float obsMin, obsMax;
	float projMin, projMax;

	UncertaintyAxisAlignment();
};
struct TargetMatchingData
{
	struct Point2DMatchCandidate
	{
		int index = -1;
		bool accepted = false;
		// Match difference value
		Eigen::Vector2f offset;
		float distSq;
		float difference;
		// Mismatch calculation using neighbouring influences
		int influenceCount = 0;
		float influence = 0; // Sum of all nearby point influences
		float similarity = 0; // Similarity [0-1] to matches of nearby points weighted by that points influence
		float mismatch;
		// Final valuie
		float value;
	};
	
	struct MarkerMatchingData
	{
		int marker;
		Eigen::Vector2f projected;
		std::vector<Point2DMatchCandidate> matches;
	};

	int identifier = -1;
	float lower, upper;
	int markerCount = 0;
	std::vector<MarkerMatchingData> markers;
	Eigen::Isometry3f pose;
};
struct TargetTracking2DData
{
	struct CameraMatchingStages
	{
		static constexpr int MaxStages = 8;
		std::array<TargetMatchingData, MaxStages> stages;
		int numStages = 0;

		void clear()
		{ // Keep allocations of each stage
			for (auto &stage : stages)
			{ // Keep allocations of markers and markers->matches
				stage.markerCount = 0;
				stage.identifier = -1;
			}
			numStages = 0;
		}
	};
	std::vector<CameraMatchingStages> matching;

	std::vector<UncertaintyAxisAlignment> uncertaintyAxis;

	TargetTracking2DData() {}
	TargetTracking2DData(int cameraCount) : uncertaintyAxis(cameraCount), matching(cameraCount) {}

	void init(int cameraCount = -1)
	{
		if (cameraCount < 0) cameraCount = uncertaintyAxis.size();
		uncertaintyAxis.clear();
		uncertaintyAxis.resize(cameraCount);
		matching.resize(cameraCount);
	}

	TargetMatchingData &nextMatchingStage(int camera, Eigen::Isometry3f pose)
	{
		assert(camera < matching.size());
		CameraMatchingStages &cam = matching[camera];
		assert(cam.numStages < CameraMatchingStages::MaxStages);
		TargetMatchingData &data = cam.stages[cam.numStages++];
		data.pose = pose;
		return data;
	}
};


/* Functions */

/**
 * Matches a relevant set of projected target points to a relevant set of point observations
 */
bool matchTargetPointsFast(
	const std::vector<Eigen::Vector2f> &points2D, const std::vector<BlobProperty> &properties, const std::vector<int> &relevantPoints2D,
	const std::vector<Eigen::Vector2f> &projected2D, const std::vector<int> &relevantProjected2D,
	std::vector<std::pair<int, int>> &matches, TargetMatchingData &matchData,
	const TargetMatchingParametersFast params, float distFactor, Eigen::Vector2f offset = Eigen::Vector2f::Zero());

/**
 * Matches a relevant set of projected target points to a relevant set of point observations
 */
void matchTargetPointsSlow(
	const std::vector<Eigen::Vector2f> &points2D, const std::vector<BlobProperty> &properties, const std::vector<int> &relevantPoints2D,
	const std::vector<Eigen::Vector2f> &projected2D, const std::vector<int> &relevantProjected2D,
	std::vector<std::pair<int, int>> &matches, TargetMatchingData &matchData,
	const TargetMatchingParametersSlow params, float distFactor, Eigen::Vector2f offset = Eigen::Vector2f::Zero());

/**
 * Optimise the given target match and update its pose, pose error and variance
 */
template<bool REFERENCE = true, bool OUTLIER = true>
TargetMatchError optimiseTargetPose(const std::vector<CameraCalib> &calibs,
	const std::vector<std::vector<Eigen::Vector2f> const *> points2D, TargetMatch2D &match, Eigen::Isometry3f prediction,
	TargetOptimisationParameters params, float errorStdDev = 0.0f, bool updateCovariance = false);

/**
 * Evaluate the given target match and update its pose error
 */
TargetMatchError evaluateTargetPose(const std::vector<CameraCalib> &calibs,
	const std::vector<std::vector<Eigen::Vector2f> const *> points2D, TargetMatch2D &match);

/**
 * Evaluate the given target match and update its pose error, and numerically calculates its covariance
 */
TargetMatchError evaluateTargetPoseCovariance(const std::vector<CameraCalib> &calibs,
	const std::vector<std::vector<Eigen::Vector2f> const *> points2D, TargetMatch2D &match, float errorStdDev);

/**
 * Updates visibleMarkers of target to those matched
 */
void updateVisibleMarkers(std::vector<std::vector<int>> &visibleMarkers, const TargetMatch2D &match);

/**
 * Redetect the target in the observed 2D points using a predicted pose
 * Iteratively matches fast, then slow if needed, optimises, matches more, and optimises
 */
TargetMatch2D trackTarget2D(const TargetCalibration3D &target, Eigen::Isometry3f prediction, const CovarianceMatrix &covariance,
	const std::vector<CameraCalib> &calibs, int cameraCount,
	const std::vector<std::vector<Eigen::Vector2f> const *> &points2D,
	const std::vector<std::vector<BlobProperty> const *> &properties,
	const std::vector<std::vector<int> const *> &relevantPoints2D,
	const TargetTrackingParameters &params, TargetTracking2DData &internalData);

#endif // TARGET_TRACKING_2D_H