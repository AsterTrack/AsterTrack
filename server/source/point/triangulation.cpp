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

//#define LOG_MAX_LEVEL LTrace

#include "point/triangulation.hpp"
#include "util/eigenutil.hpp"

#include "util/log.hpp"
#include "util/util.hpp"

#ifdef OPT_AUTODIFF
#include "unsupported/Eigen/AutoDiff"
#endif

/**
 * Triangulation of 3D points from 2D blobs
 */


/* Structures */

typedef uint8_t RayIxCnt; // Limits number of intersections per ray. Theoretically unlimited.

struct MergedIntersection
{
	Eigen::Vector3f center;
	float error;
	std::vector<BlobIndex> blobs; // blobs[cameraIndex] = blobIndex, -1 = not seen from camera
	MergedIntersection() : center(Eigen::Vector3f::Zero()), error(0.0f) {}
};

struct TwoIntersection
{
	Eigen::Vector3f center;
	float error;
	MergedIntersection *merge;
	CamIndex c1, c2;
	BlobIndex b1, b2;
	TwoIntersection(Eigen::Vector3f center, float error, CamIndex c1, BlobIndex b1, CamIndex c2, BlobIndex b2)
		: center(center), error(error), merge(nullptr), c1(c1), c2(c2), b1(b1), b2(b2) {}
};


/* Temporary data fields */

thread_local std::vector<std::vector<RayIxCnt>> rayIxCnt;


/* Functions */


static void findInitialRayIntersections(const std::vector<CameraCalib> &cameras, 
	const std::vector<std::vector<Eigen::Vector2f> const *> &points2D, const std::vector<std::vector<int> const *> &relevantPoints2D,
	std::vector<TwoIntersection> &intersections, std::vector<std::vector<RayIxCnt>> &rayIxCnt, float maxError, float minError)
{
	int camCount = cameras.size();

	// Prepare allocated memory, cast rays
	thread_local std::vector<std::vector<Ray3f>> rayGroups;
	if (rayGroups.size() < camCount)
		rayGroups.resize(camCount);
	if (rayIxCnt.size() < camCount)
		rayIxCnt.resize(camCount);
	for (int c = 0; c < camCount; c++)
	{
		rayIxCnt[c].clear();
		rayIxCnt[c].resize(points2D[c]->size(), 0);
		rayGroups[c].resize(points2D[c]->size());
		for (int p : *relevantPoints2D[c])
		{
			rayGroups[c][p] = castRay<float>(points2D[c]->at(p), cameras[c]);
		}
	}

	// Fill with candidate intersections
	for (int i = 0; i < camCount-1; i++)
	{
		for (int j = i+1; j < camCount; j++)
		{
			for (BlobIndex v : *relevantPoints2D[i])
			{
//				const Ray3f ray1 = castRay<float>(points1->at(v), cameras[i]);
				const Ray3f ray1 = rayGroups[i][v];
				for (BlobIndex w : *relevantPoints2D[j])
				{
//					const Ray3f ray2 = castRay<float>(points2->at(w), cameras[j]);
					const Ray3f ray2 = rayGroups[j][w];
					// Calculate ray intersection
					float sec1, sec2;
					getRayIntersect(ray1, ray2, &sec1, &sec2);
					if (sec1 < 0 || sec2 < 0) continue;
					Eigen::Vector3f pos1 = ray1.pos + ray1.dir * sec1;
					Eigen::Vector3f pos2 = ray2.pos + ray2.dir * sec2;
					// Calculate distance
					float errorCone = maxError*std::min(sec1*(float)cameras[i].f, sec2*(float)cameras[j].f);
					float errorSq = (pos1-pos2).squaredNorm()/4;
					if (errorSq > errorCone*errorCone) continue;
					// Increase ray intersection count
					rayIxCnt[i][v]++;
					rayIxCnt[j][w]++;
					// Register intersection
					intersections.emplace_back((pos1+pos2)/2, std::max(minError, std::sqrt(errorSq)), i, v, j, w);
					LOGC(LTrace, "2-Intersection with error %f\n", std::sqrt(errorSq));
				}
			}
		}
	}
	LOGC(LDebug, "Got %d 2-intersections!\n", (int)intersections.size());
}

void triangulateRayIntersections(const std::vector<CameraCalib> &cameras, 
	const std::vector<std::vector<Eigen::Vector2f> const *> &points2D, const std::vector<std::vector<int> const *> &relevantPoints2D,
	int cameraCount, std::vector<TriangulatedPoint> &points3D, float maxError, float minError)
{
	ScopedLogCategory scopedLogCategory(LTriangulation);

	#define IXNUM(ix) (int)(ix == NULL? -1 : (((intptr_t)ix-(intptr_t)intersections.data())/(sizeof(TwoIntersection))))

	// NOTE: This is the subset camera count, full cameraCount will only be expanded to for output
	int camCount = cameras.size();

	// Find initial set of 2-intersections
	thread_local std::vector<TwoIntersection> intersections;
	intersections.clear();
	findInitialRayIntersections(cameras, points2D, relevantPoints2D, intersections, rayIxCnt, maxError, minError);
	int ixCnt = intersections.size();

	// Have to reserve to prevent reallocation, since it relies on pointers to merged intersections
	thread_local std::vector<MergedIntersection> mergedIntersections;
	mergedIntersections.clear();
	mergedIntersections.reserve(ixCnt/2+1);

	// Merge possible intersections between three rays
	std::vector<TwoIntersection*> mergers;
	std::vector<TwoIntersection*> potentialMergers;
	std::vector<BlobIndex> ixBlobs(camCount, InvalidBlob);
	for (int i = 0; i < intersections.size(); i++)
	{
		TwoIntersection *ix = &intersections[i];

		// Check if it has been merged yet
		if (ix->merge != NULL)
		{
			//LOGC(LTrace, "------ Skipping merged intersections %d on rays %d and %d", i, ix->c1, ix->c2);
			continue;
		}

		ixBlobs[ix->c1] = ix->b1;
		ixBlobs[ix->c2] = ix->b2;

		// Search for other intersections with one common ray and one new ray
		auto testMerger = [&](TwoIntersection *ixm, int testCam)
		{
			float errorSq = (ixm->center - ix->center).squaredNorm()/4;
			float distSq = (cameras[testCam].transform.translation().cast<float>() - ix->center).squaredNorm();
			float errorCone = maxError*maxError*distSq*(float)cameras[testCam].f;
			if (errorSq <= errorCone)
			{ // Merge intersections, now consisting of three rays intersecting
				mergers.push_back(ixm);
				ixBlobs[ixm->c1] = ixm->b1;
				ixBlobs[ixm->c2] = ixm->b2;
			}
			else
			{ // else two intersections with different ray groups, but distant, so one must be wrong - register conflict
				potentialMergers.push_back(ixm);
			}
		};

		// Find matching intersections
		mergers.clear();
		potentialMergers.clear();
		for (int j = i+1; j < intersections.size(); j++)
		{
			TwoIntersection *ixm = &intersections[j];
			if (ixm->merge != NULL) continue;

			if (ixBlobs[ixm->c1] == ixm->b1)
			{
				if (ixBlobs[ixm->c2] == InvalidBlob)
					testMerger(ixm, ixm->c1 == ix->c2? ix->c1 : ix->c2);
				else if (ixBlobs[ixm->c2] == ixm->b2)
					mergers.push_back(ixm);
				// else // Technically, a conflict here COULD be better matching than an intersection already included...
			}
			else if (ixBlobs[ixm->c2] == ixm->b2)
			{
				if (ixBlobs[ixm->c1] == InvalidBlob)
					testMerger(ixm, ixm->c2 == ix->c2? ix->c1 : ix->c2);
				else if (ixBlobs[ixm->c1] == ixm->b1)
					mergers.push_back(ixm);
				// else // Technically, a conflict here COULD be better matching than an intersection already included...
			}
		}

		LOGC(LTrace, "------ Intersection %d on cameras %d and %d", i, ix->c1, ix->c2);
		if (SHOULD_LOGC(LTrace) && !potentialMergers.empty())
		{
			std::string blobsStr = "";
			for (int j = 0; j < potentialMergers.size(); j++)
				blobsStr += asprintf_s("%d, ", IXNUM(potentialMergers[j]));
			LOGC(LTrace, "Potentially conflicting/merging intersections: %s", blobsStr.c_str());
		}

		// Check if merge candidates found (only for 3 rays+)
		if (mergers.empty())
		{ // Clean up
			ixBlobs[ix->c1] = InvalidBlob;
			ixBlobs[ix->c2] = InvalidBlob;
			continue;
		}

		// Add original intersection
		mergers.push_back(ix);

		if (SHOULD_LOGC(LTrace))
		{
			std::string blobsStr = "";
			for (int j = 0; j < camCount; j++)
				blobsStr += ixBlobs[j] == InvalidBlob? "X - " : asprintf_s("%d - ", ixBlobs[j]);
			LOGC(LTrace, "Merging Blobs: %s", blobsStr.c_str());
		}

		// Go through conflicts (other intersections on the two rays of our main intersection ix respectively)
		// And find those that intersect with any two rays involved in this merging intersection
		// Then add them to the merge and remove them as conflicts
		for (int i = 0; i < potentialMergers.size(); i++)
		{
			TwoIntersection *ixm = potentialMergers[i];
			if (ixBlobs[ixm->c1] == ixm->b1 && ixBlobs[ixm->c2] == ixm->b2)
			{ // Accept as merger, probably out of error range of another set of rays tested against
				mergers.push_back(potentialMergers[i]);
				LOGC(LDebug, "Added intersection %d to merge because of shared rays!\n", IXNUM(potentialMergers[i]));
			}
		}

		if (SHOULD_LOGC(LDebug))
		{
			std::string blobsStr = "";
			for (int j = 0; j < mergers.size(); j++)
				blobsStr += asprintf_s("%d, ", IXNUM(mergers[j]));
			LOGC(LDebug, "Merging Intersections: %s", blobsStr.c_str());
		}

		// Update intersection metrics rayIxCnt and ixCnt
		for (int i = 0; i < mergers.size(); i++)
		{
			TwoIntersection *ixm = mergers[i];
			rayIxCnt[ixm->c1][ixm->b1]--;
			rayIxCnt[ixm->c2][ixm->b2]--;
		}
		for (int i = 0; i < camCount; i++)
			if (ixBlobs[i] != InvalidBlob)
				rayIxCnt[i][ixBlobs[i]]++;
		ixCnt = ixCnt - mergers.size() + 1;

		// Merge intersections properly
		assert(mergedIntersections.size()+1 < mergedIntersections.capacity()); // Otherwise, reserve metric failed
		mergedIntersections.emplace_back();
		MergedIntersection *ixm = &mergedIntersections.back();
		for (int i = 0; i < mergers.size(); i++)
		{
			mergers[i]->merge = ixm;
			ixm->center += mergers[i]->center;
			ixm->error += mergers[i]->error;
			// NOTE: This is not the true center of the merged intersection, just a quick approximation
			// refineTriangulation/refineTriangulationIterative are used later to improve it
		}
		ixm->center = ixm->center / mergers.size();
		ixm->error = ixm->error / mergers.size();
		ixm->blobs = std::move(ixBlobs);

		// Prepare ixBlobs for next iteration
		ixBlobs.clear();
		ixBlobs.resize(camCount, InvalidBlob);
	}

	// Compile all intersections as triangulated points
	points3D.reserve(ixCnt);
	auto handlePoints = [&](auto *ixm)
	{
		int clean = 0, conflict = 0;
		std::vector<BlobIndex> expandedBlobs(cameraCount, InvalidBlob);
		auto blob = [&](int c, int b)
		{
			assert(points2D[c]->size() > b);
			if (rayIxCnt[c][b] == 1) clean++;
			else conflict++;
			expandedBlobs[cameras[c].index] = b;
		};
		if constexpr (std::is_same_v<decltype(ixm), MergedIntersection*>)
		{ // MergedIntersection
			for (int j = 0; j < camCount; j++)
				if (ixm->blobs[j] != InvalidBlob)
					blob(j, ixm->blobs[j]);
		}
		else
		{ // TwoIntersection
			blob(ixm->c1, ixm->b1);
			blob(ixm->c2, ixm->b2);
		}
//		float confidence = (clean*clean)/(conflict+1);
		float confidence = clean*clean*2 + conflict;
		points3D.emplace_back(ixm->center, ixm->error, confidence);
		points3D.back().blobs = std::move(expandedBlobs);
	};
	for (int i = 0; i < mergedIntersections.size(); i++)
	{
		handlePoints(&mergedIntersections[i]);
	}
	for (int i = 0; i < intersections.size(); i++)
	{
		if (intersections[i].merge == NULL)
			handlePoints(&intersections[i]);
	}
	assert(points3D.size() == ixCnt);
}

/**
 * Pick best points for each blob conflict and reevaluate point confidences
 * Leaves points3D in a semi-sorted order, highest confidence (sec. error) first
 * Uses state from triangulation to optimise
 */
void resolveTriangulationConflicts(const std::vector<CameraCalib> &cameras, std::vector<TriangulatedPoint> &points3D, float maxError)
{
	// Sort by confidence and, secondarily, error (can also be used to punish high errors more severely)
	std::sort(points3D.begin(), points3D.end(), [maxError](const TriangulatedPoint &a, const TriangulatedPoint &b){ 
		return (a.confidence-a.error/maxError) > (b.confidence-b.error/maxError);
	});
	// could get away with no sorting by doing an additional pass over all points, but sorting speeds stuff up down the line anyways

	for (int i = 0; i < points3D.size(); i++)
	{
		int nc = 0, c = 0;
		for (int j = 0; j < cameras.size(); j++)
		{
			int c = cameras[j].index;
			int r = points3D[i].blobs[c];
			if (r != InvalidBlob)
			{
				if (rayIxCnt[j][r] == (RayIxCnt)-1)
				{ // Already claimed by a point with higher confidence
					c++;
				}
				else
				{ // Else claim it (doesn't matter if there's only one intersection on this ray)
					rayIxCnt[j][r] = (RayIxCnt)-1;
					nc++;
				}
			}
		}
		// Calculate new confidence:
		points3D[i].confidence = nc*nc*2 + c;
		LOGC(LTrace, "    Point %d: Error: %f, Confidence: %f, nc=%d, c=%d\n", i, points3D[i].error, points3D[i].confidence, nc, c);
	}
}

/**
 * Filter out points below the confidence threshold
 */
void filterTriangulatedPoints(std::vector<TriangulatedPoint> &points3D, std::vector<TriangulatedPoint> &discarded3D, float confidenceThreshold)
{
	int index = 0;
	for (int i = 0; i < points3D.size(); i++)
	{
		if (points3D[i].confidence >= confidenceThreshold)
		{
			if (index != i)
				points3D[index++] = std::move(points3D[i]);
			else 
				index++;
		}
		else 
		{
			discarded3D.push_back(std::move(points3D[i]));
		}
	}
	LOG(LTriangulation, LTrace, "%d triangulated points remaining after filtering!", index);
	points3D.resize(index, TriangulatedPoint(Eigen::Vector3f::Zero(), 0, 0)); // Not used, but no default constructor wanted
}

/**
 * Basic triangulation of point through ray intersection. The same as performed in triangulateRayIntersections
 */
template<typename Scalar, typename PointScalar, typename CalibScalar>
Eigen::Matrix<Scalar,3,1> triangulatePoint(const std::vector<std::vector<Eigen::Matrix<PointScalar,2,1>> const *> &points2D, 
	const std::vector<CameraCalib_t<CalibScalar>> &cameras, TriangulatedPoint &point3D)
{
	Eigen::Matrix<Scalar,3,1> center = Eigen::Matrix<Scalar,3,1>::Zero();
	int centerCnt = 0;
	for (int c1 = 0; c1 < cameras.size(); c1++)
	{
		BlobIndex blob1 = point3D.blobs[cameras[c1].index];
		if (blob1 == InvalidBlob) continue;
		for (int c2 = c1+1; c2 < cameras.size(); c2++)
		{
			BlobIndex blob2 = point3D.blobs[cameras[c2].index];
			if (blob2 == InvalidBlob) continue;
			Scalar sec1, sec2;
			Ray3_t<Scalar> ray1 = castRay<Scalar>(points2D[c1]->at(blob1), cameras[c1]);
			Ray3_t<Scalar> ray2 = castRay<Scalar>(points2D[c2]->at(blob2), cameras[c2]);
			getRayIntersect(ray1, ray2, &sec1, &sec2);
			center += (ray1.pos + ray1.dir*sec1 + ray2.pos + ray2.dir*sec2) / 2;
			centerCnt++;
		}
	}
	center /= centerCnt;
	point3D.pos = center.template cast<float>();
//	point3D.error =  TODO Set Error!
	return center;
}

/**
 * Refine triangulation accuracy of point by minimising the reprojection error iteratively (nearly projection invariant)
 */
template<typename Scalar, typename PointScalar, typename CalibScalar, typename TriScalar>
Eigen::Matrix<Scalar,3,1> refineTriangulationIterative(const std::vector<std::vector<Eigen::Matrix<PointScalar,2,1>> const *> &points2D, 
	const std::vector<CameraCalib_t<CalibScalar>> &cameras, TriangulatedPoint_t<TriScalar> &point3D, int maxIterations, float threshold3D)
{
	typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic> MatrixX;
	typedef Eigen::Matrix<Scalar,4,1> Vector4;
	typedef Eigen::Matrix<Scalar,3,1> Vector3;

	// Get involved cameras
	int camCount = 0;
	for (int c = 0; c < cameras.size(); c++)
		camCount += (point3D.blobs[cameras[c].index] != InvalidBlob);
	// Build base data matrix as well as row-vectors for weights
	MatrixX triSolveBase = MatrixX(camCount*2, 4);
	MatrixX thirdRow = MatrixX(camCount, 4);
	int camIndex = 0;
	for (int c = 0; c < cameras.size(); c++)
	{
		BlobIndex blob = point3D.blobs[cameras[c].index];
		if (blob == InvalidBlob) continue;
		auto camMat = cameras[c].camera.matrix().template cast<Scalar>();
		auto point = points2D[c]->at(blob).template cast<Scalar>();
		triSolveBase.row(camIndex*2+0) = point.x() * camMat.row(2) - camMat.row(0);
		triSolveBase.row(camIndex*2+1) = point.y() * camMat.row(2) - camMat.row(1);
		thirdRow.row(camIndex) = camMat.row(2);
		camIndex++;
	}
	// Solve in the least-squares sense
	// TODO: Consider using cheaper ways to solve this system, like QR decomposition, or even normal equations (really fast for many cameras)
	// https://eigen.tuxfamily.org/dox/group__LeastSquares.html
	Eigen::BDCSVD<MatrixX, Eigen::ComputeThinV> svd_tri_base(triSolveBase);
	Vector4 triResult = svd_tri_base.matrixV().rightCols(1);
	// Iteratively find better solution
	MatrixX triSolve = triSolveBase;
	Vector3 lastTri = triResult.hnormalized();
	for (int i = 1; i < maxIterations; i++)
	{
		// Adjust weights
		for (int c = 0; c < camCount; c++)
		{
			Scalar weight = (Scalar)1/thirdRow.row(c).dot(triResult);
			triSolve.row(c*2+0) = triSolveBase.row(c*2+0) * weight;
			triSolve.row(c*2+1) = triSolveBase.row(c*2+1) * weight;
		}
		// Solve in the least-squares sense
		Eigen::BDCSVD<MatrixX, Eigen::ComputeThinV> svd_tri(triSolve);
		triResult = svd_tri.matrixV().rightCols(1);
		// Abort condition: Change in reprojection error or 3D position?
		Vector3 newTri = triResult.hnormalized();
		Scalar change3D = (newTri-lastTri).norm();
		lastTri = newTri;
		if (change3D <= (Scalar)threshold3D)
		{
			LOG(LTriangulation, LTrace, "Triangulation got %f error after %d a total of iterations!\n", (triSolve * triResult).cwiseAbs().mean()*1000, i);
			break;
		}
	}
	point3D.pos = lastTri.template cast<TriScalar>();
	// Get reprojection error, not sure if properly in pixels
	//point3D.error = (triSolve * triResult).cwiseAbs().mean()*1000;
	// Actually, detection.cpp currently relies on this being "error" in m
	// So keep estimate from before
	return lastTri;
}

/**
 * Refine triangulation accuracy of point by minimising the reprojection error (not projection invariant)
 */
template<typename Scalar, typename PointScalar, typename CalibScalar>
Eigen::Matrix<Scalar,3,1> refineTriangulation(const std::vector<std::vector<Eigen::Matrix<PointScalar,2,1>> const *> &points2D, 
	const std::vector<CameraCalib_t<CalibScalar>> &cameras, TriangulatedPoint &point3D)
{
	return refineTriangulationIterative<Scalar>(points2D, cameras, point3D, 0, 0); // Not refining iterations
}

// Generate specific implementations

template Eigen::Vector3f triangulatePoint(const std::vector<std::vector<Eigen::Vector2f> const *> &points2D, 
	const std::vector<CameraCalib> &cameras, TriangulatedPoint &point3D);

template Eigen::Vector3f refineTriangulation(const std::vector<std::vector<Eigen::Vector2f> const *> &points2D, 
	const std::vector<CameraCalib> &cameras, TriangulatedPoint &point3D);
template Eigen::Vector3d refineTriangulation(const std::vector<std::vector<Eigen::Vector2d> const *> &points2D, 
	const std::vector<CameraCalib> &cameras, TriangulatedPoint &point3D);

template Eigen::Vector3f refineTriangulationIterative(const std::vector<std::vector<Eigen::Vector2f> const *> &points2D, 
	const std::vector<CameraCalib> &cameras, TriangulatedPoint_t<float> &point3D, int maxIterations, float threshold3D);
template Eigen::Vector3d refineTriangulationIterative(const std::vector<std::vector<Eigen::Vector2d> const *> &points2D, 
	const std::vector<CameraCalib> &cameras, TriangulatedPoint_t<float> &point3D, int maxIterations, float threshold3D);
template Eigen::Vector3d refineTriangulationIterative(const std::vector<std::vector<Eigen::Vector2d> const *> &points2D, 
	const std::vector<CameraCalib> &cameras, TriangulatedPoint_t<double> &point3D, int maxIterations, float threshold3D);

#ifdef OPT_AUTODIFF
// For AutoDiff used in optimisation/optimisation.hpp ()
typedef Eigen::AutoDiffScalar<VectorX<float>> ActiveScalarf;
typedef Eigen::AutoDiffScalar<VectorX<double>> ActiveScalard;
template Vector3<ActiveScalarf> refineTriangulation(const std::vector<std::vector<Vector2<ActiveScalarf>> const *> &points2D, 
	const std::vector<CameraCalib_t<ActiveScalarf>> &cameras, TriangulatedPoint &point3D);
template Vector3<ActiveScalard> refineTriangulation(const std::vector<std::vector<Vector2<ActiveScalard>> const *> &points2D, 
	const std::vector<CameraCalib_t<ActiveScalard>> &cameras, TriangulatedPoint &point3D);
template Vector3<ActiveScalarf> refineTriangulationIterative(const std::vector<std::vector<Vector2<ActiveScalarf>> const *> &points2D, 
	const std::vector<CameraCalib_t<ActiveScalarf>> &cameras, TriangulatedPoint &point3D, int maxIterations, float threshold3D);
template Vector3<ActiveScalard> refineTriangulationIterative(const std::vector<std::vector<Vector2<ActiveScalard>> const *> &points2D, 
	const std::vector<CameraCalib_t<ActiveScalard>> &cameras, TriangulatedPoint &point3D, int maxIterations, float threshold3D);
#endif