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

#include "pipeline/frameRecord.hpp"

#include "util/eigendef.hpp"

/**
 * Fitting a target (set of markers) to observed points in 2D space
 */


/* Structures */

struct TgtErrorRes
{
	float mean, stdDev, max;
	int samples;
};

struct TargetMatch2D
{
	TargetTemplate3D const *targetTemplate;
	std::vector<std::vector<std::pair<int,int>>> points2D;
	int pointCount;
	Eigen::Isometry3f pose;
	TgtErrorRes error;
};

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

	void clear()
	{ // Keep allocations of points and points->matches
		markerCount = 0;
		identifier = -1;
	}
};
struct TargetTracking2DData
{
	std::vector<UncertaintyAxisAlignment> uncertaintyAxis;
	using CameraMatchingStages = std::vector<TargetMatchingData>;
	std::vector<CameraMatchingStages> matching;

	TargetTracking2DData() {}
	TargetTracking2DData(int cameraCount) : uncertaintyAxis(cameraCount), matching(cameraCount) {}

	void init(int cameraCount = -1)
	{
		if (cameraCount < 0) cameraCount = uncertaintyAxis.size();
		uncertaintyAxis.clear();
		uncertaintyAxis.resize(cameraCount);
		matching.resize(cameraCount);
		for (auto &camera : matching)
			for (auto &stage : camera)
				stage.clear();
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
 * Optimises the pose to better fit the matched observations
 */
template<bool OUTLIER = true>
TgtErrorRes optimiseTargetPose(const std::vector<CameraCalib> &calibs,
	const std::vector<std::vector<Eigen::Vector2f> const *> points2D, TargetMatch2D &candidate,
	TargetOptimisationParameters params);

/**
 * Calculates the reprojection error of the target candidate to the matched observations
 */
TgtErrorRes calculateTargetErrors(const std::vector<CameraCalib> &calibs,
	const std::vector<std::vector<Eigen::Vector2f> const *> points2D, TargetMatch2D &targetMatch2D);

/**
 * Updates visibleMarkers of target to those matched
 */
void updateVisibleMarkers(std::vector<std::vector<int>> &visibleMarkers, const TargetMatch2D &match2D);

/**
 * Redetect the target in the observed 2D points using a predicted pose
 * First re-aquires the previously visible markers and finds a better pose with them,
 * then finds newly appearing markers and optimises the pose to fit all matched observations
 */
TargetMatch2D trackTarget2D(const TargetTemplate3D &target, Eigen::Isometry3f pose, Eigen::Vector3f stdDev,
	const std::vector<CameraCalib> &calibs, int cameraCount,
	const std::vector<std::vector<Eigen::Vector2f> const *> &points2D,
	const std::vector<std::vector<BlobProperty> const *> &properties,
	const std::vector<std::vector<int> const *> &relevantPoints2D,
	const TargetTrackingParameters &params, TargetTracking2DData &internalData);

#endif // TARGET_TRACKING_2D_H