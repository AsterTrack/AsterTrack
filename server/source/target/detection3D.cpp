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

#include "target/detection3D.hpp"
#include "util/eigenalg.hpp"

#include "util/log.hpp"

#include <cassert>

/**
 * Detecting a target (set of markers) in a triangulated 3D point cloud
 */


/* Functions */

/**
 * Calculate MSE of candidate marker in given point cloud in mm^2
 */
float calculateCandidateMSE(const TargetTemplate3D &target3D, 
	const std::vector<TriangulatedPoint> &points3D, const TargetCandidate3D &candidate)
{
	float meanSquaredError = 0.0f;
	for (int j = 0; j < candidate.points.size(); j++)
	{
		int trPt = candidate.points[j];
		int mkPt = candidate.pointMap[trPt];
		Eigen::Vector3f trPosGT = points3D[trPt].pos;
		Eigen::Vector3f trPosRC = candidate.pose * target3D.markers[mkPt].pos;
		meanSquaredError += (trPosRC - trPosGT).squaredNorm();
	}
	return meanSquaredError / candidate.points.size() * (1000*1000);
}

/**
 * Picks the best candidate by point count and internal MSE.
 * Returns the index 0f best and count of other candidates with the same maximum point count
 * Calculates MSE in mm^2 of those best candidates
 */
std::tuple<int,int> getBestTargetCandidate(const TargetTemplate3D &target3D, 
	const std::vector<TriangulatedPoint> &points3D, std::vector<TargetCandidate3D> &candidates)
{
	// Find maximum point count and how many candidates have this point count
	int bestCandPtCnt = -1, bestCandCount = 0;
	for (int i = 0; i < candidates.size(); i++)
	{
		if (bestCandPtCnt < (int)candidates[i].points.size())
		{
			bestCandPtCnt = candidates[i].points.size();
			bestCandCount = 1;
		}
		else if (bestCandPtCnt == candidates[i].points.size())
			bestCandCount++;
	}

	// Find the best candidate with maximum point count and best internal MSE
	int best = -1;
	for (int i = 0; i < candidates.size(); i++)
	{
		TargetCandidate3D &candidate = candidates[i];
		if (candidate.points.size() == bestCandPtCnt)
		{ // Calculate mean square error
			candidate.MSE = calculateCandidateMSE(target3D, points3D, candidate);
			if (best < 0 || candidate.MSE < candidates[best].MSE)
				best = i;
		}
	}
	assert(candidates.empty() || best >= 0);

	return { best, bestCandCount };
}

/**
 * Records relevant point relations between triangulated points such that:
 * - Each relation is registered twice, as a base and arm version, on both involved points
 * - The base is registered with the lower index point, the arm with the higher index point
 * - The point it is registered with comes first (pt1), then the partner (pt2)
 * - For each point, trRelations contains all bases first, then all arms
 */
void createPointNetwork(const std::vector<TriangulatedPoint> &points3D, const std::vector<int> &indices,
	float maxRelevantDistance, 
	std::vector<int> &trRelBases, std::vector<std::vector<PointRelation>> &trRelations)
{
	float maxRelevantDistanceSq = maxRelevantDistance*maxRelevantDistance;
	trRelBases.resize(points3D.size());
	trRelations.resize(points3D.size());
	for (int ii = 0; ii < indices.size(); ii++)
	{
		int i = indices[ii];
		for (int jj = ii+1; jj < indices.size(); jj++)
		{
			int j = indices[jj];
			Eigen::Vector3f dir = points3D[j].pos - points3D[i].pos;
			//if (dir.sum() > maxRelevantDistance) continue;
			float distSq = dir.squaredNorm();
			if (distSq < maxRelevantDistanceSq)
			{
				float dist = std::sqrt(distSq);
				dir = dir / dist; // Normalise direction
				trRelations[i].emplace_back(i, j,  dir, dist);
				LOGC(LTrace, "Relation (%d - %d)\n", i, j);
				trRelBases[i]++;
			}
		}
	}
	// Duplicate relations as arms so that all relations with pt1<pt2 come first
	for (int i : indices)
	{
		for (int j = 0; j < trRelBases[i]; j++)
		{
			PointRelation &rel = trRelations[i][j];
			trRelations[rel.pt2].emplace_back(rel.pt2, rel.pt1, -rel.dir, rel.distance);
		}
	}
}

/**
 * Detect a marker in the triangulated 3D Point cloud
 * Returns all candidates
 */
void detectTarget3D(const TargetTemplate3D &target3D,
	const std::vector<TriangulatedPoint> &points3D, const std::vector<int> &indices,
	std::vector<TargetCandidate3D> &candidates,
	float sigmaError, float poseSigmaError, bool quickAssign)
{
	ScopedLogCategory scopedLogCategory(LDetection3D);

	// Goal is to find a target in the point cloud
	// This is done by iterating over pairs of points as bases
		// Verified by checking the target for pairs of similar distance
	// And then finding arms to compose a trio of points that has not been checked before
		// Verified by angle and similarly, distance
	// Relations are the index of short-distance point neighbours
		// Each relation is registered twice, as a base and arm version, on both involved points
		// The base is registered with the lower index point, the arm with the higher index point
		// The point it is registered with comes first (pt1), then the partner (pt2)
		// For each point, trRelations contains all bases first, then all arms
	// The idea for optimisation:
		// points3D are sorted from most to least confident
		// The expectation is that the first couple points belong to the target
		// The first bases are likely part of the target

	// TODO: For use in quickly detecting a target, use only as an initial guess generator
	// E.g. use quickAssign, then try with trackTarget2D
	// repeat that for all targets applicable, if none match, perhaps attempt full search? (or more likely, don't bother)
	// But remember, this function is also used for registration between e.g. two partially calibrated targets in target calibration

	// Get closest neighbouring points for each point
	// TODO: Outsource as preprocessing step, once for each cluster3D
	// That way, when searching for multiple targets, this step has to be done only once, and on less data (already clustered)
	float maxRelevantDistance = target3D.relationDist.size() == 0? 0.0f : 
		target3D.relationDist[target3D.relationDist.size()-1].distance*1.2f + 1.0f;
	std::vector<int> trRelBases;
	std::vector<std::vector<PointRelation>> trRelations;
	createPointNetwork(points3D, indices, maxRelevantDistance, trRelBases, trRelations);
	// Note: Indices is implemented in a transparent way
	// Meaning, memory is still generally allocated for all triangulated points
	// But in computation, they are skipped

	// Then find candidates:
	// Find 3 points in triangulated point cloud and check against matches in marker based on length and then angle
	// Calculate initial pose using kabsch algorithm on any 3 point candidate
	// Iterate over unchecked points in both clouds and include in candidate if match is found
	// Calculate pose again using kabsch algorithm

	// Helper state
	thread_local std::vector<bool> trPtAssigned;
	thread_local std::vector<bool> mkPtAssigned;
	trPtAssigned.clear();
	trPtAssigned.resize(points3D.size());
	
	// Subroutine determining candidate pose using kabsch algorithm
	auto determineCandidatePose = [&points3D](const TargetTemplate3D &target3D, TargetCandidate3D &candidate)
	{
		int ptCount = candidate.points.size();
		Eigen::MatrixX3f trMat(ptCount, 3);
		Eigen::Matrix3Xf mkMat(3, ptCount);
		for (int j = 0; j < ptCount; j++)
		{
			int trPt = candidate.points[j];
			int mkPt = candidate.pointMap[trPt];
			trMat.row(j) = points3D[trPt].pos;
			mkMat.col(j) = target3D.markers[mkPt].pos;
		}
		candidate.pose = kabsch<float,Eigen::Isometry>(trMat, mkMat);
	};
	
	// Subroutine checking the given triangulated point combination for a match in the marker, already given the possible base matches
	auto check3PointCandidate = [&](
		const TargetTemplate3D &target3D, const PointRelation &trRelBase, const PointRelation &trRelArm,
		int trPtJ, int trPtB, int trPtA,
		float baseError, float armError,
		auto mkRelBaseRange)
	{
		for (auto mkRelBase = mkRelBaseRange.first; mkRelBase < mkRelBaseRange.second; mkRelBase++)
		{ // Iterate over all matching base relation candidates
			int mkPts[2] = { mkRelBase->pt1, mkRelBase->pt2 };
			LOGC(LTrace, "|  _ - Mk Base Relation (%d - %d) of length %f!\n", mkRelBase->pt1, mkRelBase->pt2, mkRelBase->distance);

			// Since we don't know the direction of the marker relation, both sides need to be checked
			for (int m = 0; m < 2; m++)
			{
				int mkPtJ = mkPts[m];
				int mkPtB = mkPts[1-m];

				// Now candidates for the arm
				for (int n = 0; n < target3D.pointRelation[mkPtJ].size(); n++)
				{
					const PointRelation &mkRelArm = target3D.relationDist[target3D.pointRelation[mkPtJ][n]];
					int mkPtA = mkRelArm.pt1 == mkPtJ? mkRelArm.pt2 : mkRelArm.pt1;
					if (mkPtA == mkPtB) continue;

					if (std::abs(trRelArm.distance - mkRelArm.distance) < armError)
					{ // Found potential arm with same distance
						LOGC(LTrace, "|  |  _ - Mk Arm Relation (%d - %d) with length %f (error %f +- %f)!\n", 
							mkRelArm.pt1, mkRelArm.pt2, mkRelArm.distance, 
							std::abs(trRelArm.distance - mkRelArm.distance), armError);

						// Make sure they have the same angle
						int mkDirFlip = mkRelBase->pt1 == mkRelArm.pt1 || mkRelBase->pt2 == mkRelArm.pt2? 1 : -1;
						float mkAngle = std::acos(mkDirFlip * mkRelBase->dir.dot(mkRelArm.dir));
						int trDirFlip = trRelBase.pt1 == trRelArm.pt1 || trRelBase.pt2 == trRelArm.pt2? 1 : -1;
						float trAngle = std::acos(trDirFlip * trRelBase.dir.dot(trRelArm.dir));
						float angleErrorLimit = std::asin(baseError / mkRelBase->distance) + std::asin(armError / mkRelArm.distance);
						if (std::abs(trAngle-mkAngle) > angleErrorLimit)
						{
							LOGC(LTrace, "|  |  |    - Expected angle %f but is %f!\n", mkAngle/PI*180.0f, trAngle/PI*180.0f);
							continue;
						}
						// Initialise target candidate - candidate pose could now be extracted
						TargetCandidate3D candidate = {};
						candidate.pointMap.resize(points3D.size(), -1);
						// Add initial three points
						candidate.points.push_back(trPtA);
						candidate.pointMap[trPtA] = mkPtA;
						candidate.points.push_back(trPtJ);
						candidate.pointMap[trPtJ] = mkPtJ;
						candidate.points.push_back(trPtB);
						candidate.pointMap[trPtB] = mkPtB;
						// Debug
						LOGC(LDebug, "|  |  |  _ - Candidate %d: (%d->%d), (%d->%d), (%d->%d)\n", 
							(int)candidates.size(), trPtA, mkPtA, trPtJ, mkPtJ, trPtB, mkPtB);
						LOGC(LTrace, "|  |  |  _ -  Angle %f %+f (+-%f); 1-2 %f %+f (+-%f); 2-3 %f %+f (+-%f)\n", 
							mkAngle/PI*180.0f, (trAngle-mkAngle)/PI*180.0f, angleErrorLimit/PI*180.0f,
							mkRelBase->distance, (trRelBase.distance-mkRelBase->distance), baseError,
							mkRelArm.distance, (trRelArm.distance-mkRelArm.distance), armError);

						// ----- Add more points -----

						// Calculate estimated pose of the three points
						determineCandidatePose(target3D, candidate);

						// Prepare assigned for marker points, used to avoid duplicate checks
						mkPtAssigned.clear();
						mkPtAssigned.resize(target3D.markers.size());
						for (int o = 0; o < candidate.points.size(); o++)
							mkPtAssigned[candidate.pointMap[candidate.points[o]]] = true;

						// Find extends of the candidate
						Eigen::Vector3f com = Eigen::Vector3f::Zero();
						for (int o = 0; o < candidate.points.size(); o++)
						{
							int trPt = candidate.points[o];
							com += points3D[trPt].pos;
						}
						com = com/candidate.points.size();
						float poseErrorSqNew = 0;
						float poseErrorSqOld = 0;
						for (int o = 0; o < candidate.points.size(); o++)
						{
							int trPt = candidate.points[o];
							float distSq = (points3D[trPt].pos-com).squaredNorm();
							poseErrorSqNew += distSq / (points3D[trPt].error*points3D[trPt].error);
							poseErrorSqOld += points3D[trPt].error*points3D[trPt].error / distSq;
						}
						poseErrorSqNew = 1.0f / poseErrorSqNew;
						poseErrorSqOld = poseErrorSqOld/candidate.points.size();

						float poseErrorSq = poseErrorSqNew * poseSigmaError*poseSigmaError;
						if (poseErrorSqNew > 0.03*0.03)
						//if (poseErrorSqOld > 0.25*0.25)
						{ // Error too high, even if this is a valid candidate
							// This is just because the chosen points are too close together, and thus angular uncertainty is higher
							LOGC(LDarn, "|  |  |  |    - Discarding 3-point candidate, inherent pose error %f too high! (Old rating %f)", std::sqrt(poseErrorSqNew), std::sqrt(poseErrorSqOld));
							for (int o = 0; o < candidate.points.size(); o++)
							{
								int trPt = candidate.points[o];
								float dist = (points3D[trPt].pos-com).norm();
								float error = points3D[trPt].error / dist;
								LOGC(LDebug, "|  |  |  |  |    - Point %d, dist %fmm, error %f, rel error %f", trPt, dist*1000, points3D[trPt].error, error);
							}
							return;
						}
						float poseError = std::sqrt(poseErrorSq); 
						 // = poseSigmaError * 1/sum(distFromCoM / pointError)

						// Brute force check all points
						Eigen::Isometry3f invTransform = candidate.pose.inverse(); // Very cheap inverse
						for (int j : indices)
						{
							if (trPtAssigned[j]) continue;
							if (candidate.pointMap[j] >= 0) continue;
							Eigen::Vector3f estMkPos = invTransform * points3D[j].pos;
							float ptError = sigmaError * points3D[j].error;
							//float distFromCoM = (points3D[j].pos - com).norm();
							//float errorLimit = ptError + poseError * distFromCoM;
							float distFromCoMSq = (points3D[j].pos - com).squaredNorm();
							float errorLimitSq = ptError*ptError + poseErrorSq*distFromCoMSq + 2*poseError*ptError;
							//float errorLimitSq = ptError*ptError;
							for (int k = 0; k < target3D.markers.size(); k++)
							{
								if (mkPtAssigned[k]) continue;
								float errorSq = (estMkPos - target3D.markers[k].pos).squaredNorm();
								if (errorSq < errorLimitSq)
								{
									LOGC(LDebug, "|  |  |  |    - Added (%d - %d) - dist %f, max %f!\n", 
										j, k, std::sqrt(errorSq), std::sqrt(errorLimitSq));
									LOGC(LTrace, "|  |  |  |    -   pt %f pose %f, dist %f)!\n", 
										ptError, std::sqrt(poseErrorSq), std::sqrt(distFromCoMSq));
									candidate.points.push_back(j);
									candidate.pointMap[j] = k;
									mkPtAssigned[k] = true;
									break;
								}
								else if (errorSq < errorLimitSq*5)
								{
									LOGC(LTrace, "|  |  |  |    - Rejected (%d - %d) - dist %f, max %f!\n", 
										j, k, std::sqrt(errorSq), std::sqrt(errorLimitSq));
									LOGC(LTrace, "|  |  |  |    -   pt %f pose %f, dist %f)!\n", 
										ptError, std::sqrt(poseErrorSq), std::sqrt(distFromCoMSq));
								}
							}
						}

						if (candidate.points.size() > 4 && quickAssign)
						{ // Candidate is almost certain, mark points as assigned
							for (int o = 0; o < candidate.points.size(); o++)
								trPtAssigned[candidate.points[o]] = true;
						}

						// Calculate final candidate pose
						if (candidate.points.size() > 3)
							determineCandidatePose(target3D, candidate);

						// Add candidate to list
						candidates.push_back(std::move(candidate));

						// One candidate for these three points is enough
						return;
					}
					else if (std::abs(trRelArm.distance - mkRelArm.distance) < armError*2)
					{ // Found potential arm with same distance
						LOGC(LTrace, "|  |  _ - Dropped Mk Arm Relation (%d - %d) with length %f (error %f +- %f)!\n", 
							mkRelArm.pt1, mkRelArm.pt2, mkRelArm.distance, std::abs(trRelArm.distance - mkRelArm.distance), armError);
					}
				}
			}
		}
	};

	// Subroutine to check if a ptA has a base (excluding arms) to ptB
	auto hasBaseTo = [&trRelations, &trRelBases](int ptA, int ptB)
	{
		bool sharedArm = false;
		for (int l = 0; l < trRelBases[ptA]; l++)
			if ((sharedArm = (trRelations[ptA][l].pt2 == ptB)))
				break;
		return sharedArm;
	};
	// Subroutine to check if a ptA has an arm (excluding bases) to ptB
	auto hasArmTo = [&trRelations, &trRelBases](int ptA, int ptB)
	{
		bool sharedArm = false;
		for (int l = trRelBases[ptA]; l < trRelations[ptA].size(); l++)
			if ((sharedArm = (trRelations[ptA][l].pt2 == ptB)))
				break;
		return sharedArm;
	};

	// Find 3 point combinations to check
	for (int i : indices)
	{
		if (trPtAssigned[i]) continue;
		// Point i is our current triangulated joint point

		for (int j = 0; j < trRelations[i].size(); j++)
		{
			const PointRelation &trRelBase = trRelations[i][j];
			if (trRelBase.pt2 < trRelBase.pt1) continue; // => trRelBase.pt1 == i
			if (trPtAssigned[trRelBase.pt2]) continue;
			// trRelBase is a relation in the triangulated point cloud which has not been used as a base before

			// Find potential matches for the base using the marker lookup table with distance within the error range
			float baseError = sigmaError * (points3D[trRelBase.pt1].error + points3D[trRelBase.pt2].error);
			auto mkRelBaseRange = std::equal_range(target3D.relationDist.begin(), target3D.relationDist.end(), 
													trRelBase.distance, ErrorRangeComp(baseError));
			LOGC(LTrace, "BASE (%d - %d) of length %f += %f: %d potential matches in marker!\n", trRelBase.pt1, trRelBase.pt2, 
				trRelBase.distance, baseError, (int)std::distance(mkRelBaseRange.first, mkRelBaseRange.second));
			if (mkRelBaseRange.first >= mkRelBaseRange.second) continue;
			// Found some possible candidates within error range

			// Check in all unchecked bases and all arms of major base point
			for (int k = j+1; k < trRelations[trRelBase.pt1].size(); k++)
			{
				const PointRelation &trRelArm = trRelations[trRelBase.pt1][k]; // trRelBase.pt1 == trRelArm.pt1
				if (trPtAssigned[trRelArm.pt2]) continue;

				if (trRelArm.pt2 < trRelArm.pt1) // was a base already handled
				{
					if (hasArmTo(trRelBase.pt2, trRelArm.pt2)) continue;
				}

				// Now we have a set of three points around joint point trRelBase->pt1 which have not been handled before
				assert(trRelBase.pt1 == trRelArm.pt1);

				float armError = sigmaError * (points3D[trRelArm.pt1].error + points3D[trRelArm.pt2].error);
				LOGC(LTrace, "_ - Tr Arm Relation (%d - %d) with length %f += %f!\n", trRelArm.pt1, trRelArm.pt2, trRelArm.distance, armError);

				LOGC(LTrace, "  -- Checking candidate Base (%d,%d)[%d] with joint arm (%d,%d)[%d]\n", 
					trRelBase.pt1, trRelBase.pt2, j, trRelArm.pt1, trRelArm.pt2, k);
				check3PointCandidate(target3D,
					trRelBase, trRelArm,
					trRelBase.pt1, trRelBase.pt2, trRelArm.pt2,
					baseError, armError,
					mkRelBaseRange);

				if (trPtAssigned[trRelBase.pt1]) break; // if true an almost certain candidate with 5+ points has been found
			}
			if (trPtAssigned[trRelBase.pt1]) break; // if true an almost certain candidate with 5+ points has been found

			// Check in arms of minor base point, IF they fulfill certain conditions
			for (int k = trRelBases[trRelBase.pt2]; k < trRelations[trRelBase.pt2].size(); k++)
			{
				const PointRelation &trRelArm = trRelations[trRelBase.pt2][k]; // trRelBase.pt2 == trRelArm.pt1
				if (trPtAssigned[trRelArm.pt2]) continue;

				// Make sure no duplicates are checked
				if (trRelArm.pt2 <= trRelBase.pt1) continue;
				if (hasBaseTo(trRelBase.pt1, trRelArm.pt2)) continue;

				// Now we have a set of three points around joint point trRelBase->pt2 which have not been handled before
				assert(trRelBase.pt2 == trRelArm.pt1);

				float armError = sigmaError * (points3D[trRelArm.pt1].error + points3D[trRelArm.pt2].error);
				LOGC(LTrace, "_ - Tr Arm Relation (%d - %d) with length %f += %f!\n", trRelArm.pt1, trRelArm.pt2, trRelArm.distance, armError);

				LOGC(LTrace, "  -- Checking candidate Base (%d,%d)[%d] with joint arm (%d,%d)[%d]\n", 
					trRelBase.pt1, trRelBase.pt2, j, trRelArm.pt1, trRelArm.pt2, k);
				check3PointCandidate(target3D,
					trRelBase, trRelArm,
					trRelBase.pt2, trRelBase.pt1, trRelArm.pt2,
					baseError, armError,
					mkRelBaseRange);

				if (trPtAssigned[trRelBase.pt1]) break; // if true an almost certain candidate with 5+ points has been found
			}
			if (trPtAssigned[trRelBase.pt1]) break; // if true an almost certain candidate with 5+ points has been found
		}
	}
}

/**
 * Detect a marker in the triangulated 3D Point cloud and sets the best candidate
 * Returns it's MSE in mm^2 (or none with point-count 0 if none found)
 */
TargetCandidate3D detectTarget3D(const TargetTemplate3D &target3D,
	const std::vector<TriangulatedPoint> &points3D, const std::vector<int> &indices,
	float sigmaError, float poseSigmaError, bool quickAssign)
{
	std::vector<TargetCandidate3D> candidates;
	detectTarget3D(target3D, points3D, indices, candidates, sigmaError, poseSigmaError, quickAssign);

	std::tuple<int,int> bestCand = getBestTargetCandidate(target3D, points3D, candidates);
	int index = std::get<0>(bestCand);
	if (index >= 0)
		return candidates[index];
	return {};
}

/**
 * Detect a marker in the triangulated 3D Point cloud
 * Rreturns all candidates with respective MSE in mm^2 and point count
 */
void detectTarget3D(const TargetTemplate3D &target3D,
	const std::vector<TriangulatedPoint> &points3D, const std::vector<int> &indices, 
	std::vector<Eigen::Isometry3f> &poses3D, std::vector<std::pair<float,int>> &posesMSE,
	float sigmaError, float poseSigmaError, bool quickAssign)
{
	std::vector<TargetCandidate3D> candidates;
	detectTarget3D(target3D, points3D, indices, candidates, sigmaError, poseSigmaError, quickAssign);

	// Calculate MSE of candidates and register
	poses3D.reserve(poses3D.size() + candidates.size());
	posesMSE.reserve(posesMSE.size() + candidates.size());
	for (int i = 0; i < candidates.size(); i++)
	{
		// Register candidate
		const TargetCandidate3D &candidate = candidates[i];
		poses3D.push_back(candidate.pose);
		posesMSE.push_back({ calculateCandidateMSE(target3D, points3D, candidate), (int)candidate.points.size() });
	}
}