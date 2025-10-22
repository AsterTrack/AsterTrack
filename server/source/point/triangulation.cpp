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

#include "point/triangulation.hpp"
#include "util/eigenutil.hpp"

#include "util/log.hpp"

#ifdef OPT_AUTODIFF
#include "unsupported/Eigen/AutoDiff"
#endif

/**
 * Triangulation of 3D points from 2D blobs
 */


/* Structures */

typedef uint8_t RayIxCnt; // Based on the maximum blob count per camera, currently 254 (-1 is unused)

struct Intersection
{
	Eigen::Vector3f center;
	float error;
	Intersection *merge;
	std::vector<BlobIndex> blobs; // blobs[cameraIndex] = blobIndex, -1 = not seen from camera
	Intersection (Eigen::Vector3f Center, float Error, int CamCount) : center(Center), error(Error), merge(NULL)
	{
		blobs.resize(CamCount, InvalidBlob);
	}
};


/* Temporary data fields */

thread_local std::vector<std::vector<RayIxCnt>> rayIxCnt;


/* Functions */


/**
 * Calculate triangulatedPoints as the intersection points between rays of each camera 
 * Calculates mean error of triangulated points to rays and confidence based on rays involved
 * With unconflicted (NC) and conflicted (C) involved rays, point confidence is 2*nc^2 + c
 * Stores intersection data internally for later use in conflict resolving
 */
void triangulateRayIntersections(const std::vector<CameraCalib> &cameras, 
	const std::vector<std::vector<Eigen::Vector2f> const *> &points2D, const std::vector<std::vector<int> const *> &relevantPoints2D,
	std::vector<TriangulatedPoint> &points3D, float maxError, float minError)
{
	ScopedLogCategory scopedLogCategory(LTriangulation);

	thread_local std::vector<Intersection> intersections;
	thread_local std::vector<Intersection> mergedIntersections;
	thread_local std::vector<std::vector<Ray3f>> rayGroups;

	#define IXNUM(ix) (int)(ix == NULL? -1 : (((intptr_t)ix-(intptr_t)intersections.data())/(sizeof(Intersection))))

	int camCount = cameras.size();
	//maxError = maxError/100; // Error at one meter distance

	// Prepare allocated memory, cast rays
	intersections.clear();
	mergedIntersections.clear();
	rayGroups.resize(camCount);
	rayIxCnt.resize(camCount);
	for (int c = 0; c < camCount; c++)
	{
		rayIxCnt[c].resize(points2D[c]->size());
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
					intersections.emplace_back((pos1+pos2)/2, std::max(minError, std::sqrt(errorSq)), camCount);
					intersections.back().blobs[i] = v;
					intersections.back().blobs[j] = w;
					LOGC(LDebug, "2-Intersection with error %f\n", std::sqrt(errorSq));
				}
			}
		}
	}
	LOGC(LDebug, "Got %d 2-intersections!\n", (int)intersections.size());

	// Have to reserve to prevent reallocation, since it relies on pointers to merged intersections
	mergedIntersections.reserve(intersections.size()/3);
	int ixCnt = intersections.size();

	// Merge possible intersections between three rays
	std::vector<Intersection*> mergers;
	std::vector<Intersection*> potentialMergers;
	std::vector<BlobIndex> ixBlobs(camCount, InvalidBlob);
	for (int i = 0; i < intersections.size(); i++)
	{
		Intersection *ix = &intersections[i];
		// Check if it has been merged yet
		if (ix->merge != NULL)
		{
			LOGC(LTrace, "------ Skipping merged intersections %d on rays ", i);
			for (int j = 0; j < camCount; j++)
				LOGCONTC(LTrace, "%d - ", ix->blobs[j]);
			continue;
		}
		// Find the indices of the two intersecting rays
		int r1 = 0;
		while (ix->blobs[r1] == InvalidBlob) r1++;
		int r2 = r1+1;
		while (ix->blobs[r2] == InvalidBlob) r2++;
		// Search for other intersections with one common ray and one new ray
		auto isWithinMargin = [&](Intersection *ixm, int testCam)
		{
			float errorSq = (ixm->center - ix->center).squaredNorm()/4;
			float distSq = (cameras[testCam].transform.translation().cast<float>() - ix->center).squaredNorm();
			float errorCone = maxError*maxError*distSq*(float)cameras[testCam].f;
			return errorSq <= errorCone;
		};
		for (int j = i+1; j < intersections.size(); j++)
		{
			Intersection *ixm = &intersections[j];
			if (ixm->merge != NULL) continue;
			// TODO: Problem: This only considers 2-Intersections that share a ray.
			// This breaks down for even just 3 cameras, though there the remaining 2-intersection is just ignored later on
			// For 4+ cameras, this will create a full alternative TriangulatedPoint, with lower score, less cameras, essentially garbage
			// But the main point at least does include all cameras and is correct
			if (ixm->blobs[r1] == ix->blobs[r1])
			{ // Intersection on same ray, either merge or create conflict
				if (ixm->blobs[r2] == InvalidBlob)
				{ // Intersection is with a different ray group, check proximity to determine if merge or conflict
					if (isWithinMargin(ixm, r2))
					{ // Merge intersections, now consisting of three rays intersecting
						mergers.push_back(ixm);
						continue;
					} // else two intersections with different ray groups, but distant, so one must be wrong
					// Register conflict
					potentialMergers.push_back(ixm);
				} // else two intersections with same ray group and definitely a conflict
			}
			else if (ixm->blobs[r2] == ix->blobs[r2])
			{ // Intersection on same ray, either merge or create conflict
				if (ixm->blobs[r1] == InvalidBlob)
				{ // Intersection is with a different ray group, check proximity to determine if merge of conflict
					if (isWithinMargin(ixm, r1))
					{ // Merge intersections, now consisting of three rays intersecting
						mergers.push_back(ixm);
						continue;
					} // else two intersections with different ray groups, but distant, so one must be wrong
					// Register conflict
					potentialMergers.push_back(ixm);
				} // else two intersections with same ray group and definitely a conflict
			}
		}

		LOGC(LTrace, "------ Intersection %d on rays ", i);
		for (int j = 0; j < camCount; j++)
			LOGCONTC(LTrace, "%d - ", ix->blobs[j]);
		LOGC(LTrace, "Potentially conflicting/merging intersections: ");
		for (int j = 0; j < potentialMergers.size(); j++)
			LOGCONTC(LTrace, "%d, ", IXNUM(potentialMergers[j]));

		// Check if merge candidates found (only for 3 rays+)
		if (mergers.size() > 0)
		{ // Merge and add new intersection
			mergers.push_back(ix);

			// In some cases, an intersection of 3 or more rays includes intersections out of error range of the others
			// They have to be manually added

// Begin OOR fix
			// Get all rays involved in this intersection (more than 2, else it wouldn't need to merge)
			for (int i = 0; i < mergers.size(); i++)
				for (int j = 0; j < camCount; j++)
					if (mergers[i]->blobs[j] != InvalidBlob)
						ixBlobs[j] = mergers[i]->blobs[j];

			LOGC(LTrace, "Involved Rays: ");
			for (int j = 0; j < camCount; j++)
				LOGCONTC(LTrace, "%d - ", ixBlobs[j]);

			// Go through conflicts (other intersections on the two rays of our main intersection ix respectively)
			// And find those that intersect with any two rays involved in this merging intersection
			// Then add them to the merge and remove them as conflicts
			for (int i = 0; i < potentialMergers.size(); i++)
			{
				bool match = true;
				for (int j = 0; j < camCount; j++)
				{
					if (potentialMergers[i]->blobs[j] != InvalidBlob && potentialMergers[i]->blobs[j] != ixBlobs[j])
					{
						match = false;
						break;
					}
				}
				if (match)
				{ // Accept as merger, probably out of error range
					mergers.push_back(potentialMergers[i]);
					LOGC(LDebug, "Added intersection %d to merge because of shared rays!\n", IXNUM(potentialMergers[i]));
				}
			}
// End OOR fix

			LOGC(LTrace, "Merging: ");
			for (int j = 0; j < mergers.size(); j++)
				LOGCONTC(LTrace, "%d - ", IXNUM(mergers[j]));

			// Merge
			mergedIntersections.emplace_back(Eigen::Vector3f::Zero(), 0.0f, camCount);
			Intersection *ixm = &mergedIntersections.back();
			for (int i = 0; i < mergers.size(); i++)
			{
				// Mark as merged
				mergers[i]->merge = ixm;
				// Update merge center
				ixm->center += mergers[i]->center;
				// Somehow update error
				ixm->error += mergers[i]->error;
				// NOTE: This is not the true center of the merged intersection, just a quick approximation
				// refineTriangulation/refineTriangulationIterative are used later to improve it
			}
			// Average out values
			ixm->center = ixm->center / mergers.size();
			ixm->error = ixm->error / mergers.size();
			// Correct error by number of involved intersections
			ixm->error = ixm->error * 2 / mergers.size();
			// Make sure involved rays are accurate
			for (int j = 0; j < camCount; j++)
				ixm->blobs[j] = ixBlobs[j];

			// Update intersection counters
			// Remove mergers
			for (int i = 0; i < mergers.size(); i++)
				for (int j = 0; j < camCount; j++)
					if (mergers[i]->blobs[j] != InvalidBlob)
						rayIxCnt[j][mergers[i]->blobs[j]]--;
			// Add merged intersection
			for (int i = 0; i < camCount; i++)
				if (ixBlobs[i] != InvalidBlob)
					rayIxCnt[i][ixBlobs[i]]++;
			// Update intersection count
			ixCnt = ixCnt-mergers.size()+1;

			// Reset for next iteration
			mergers.clear();
		}
		potentialMergers.clear();
	}

	// Compile all intersections as triangulated points
	points3D.reserve(ixCnt);
	auto handlePoints = [&](Intersection *ix) {
		int c = 0, nc = 0;
		for (int j = 0; j < camCount; j++)
		{
			int r = ix->blobs[j];
			if (r == InvalidBlob) continue;
			if (rayIxCnt[j][r] == 1) nc++;
			else c++;
		}
//		float confidence = (nc*nc)/(c+1);
		float confidence = nc*nc*2 + c;
		points3D.emplace_back(ix->center, ix->error, confidence);
		points3D.back().blobs.swap(ix->blobs);
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
}

/**
 * Pick best points for each blob conflict and reevaluate point confidences
 * Leaves points3D in a semi-sorted order, highest confidence (sec. error) first
 * Uses state from triangulation to optimise
 */
void resolveTriangulationConflicts(std::vector<TriangulatedPoint> &points3D, float maxError)
{
	// Sort by confidence and, secondarily, error (can also be used to punish high errors more severely)
	std::sort(points3D.begin(), points3D.end(), [maxError](const TriangulatedPoint &a, const TriangulatedPoint &b){ 
		return (a.confidence-a.error/maxError) > (b.confidence-b.error/maxError);
	});
	// could get away with no sorting by doing an additional pass over all points, but sorting speeds stuff up down the line anyways

	for (int i = 0; i < points3D.size(); i++)
	{
		int nc = 0, c = 0;
		for (int j = 0; j < points3D[i].blobs.size(); j++)
		{
			int r = points3D[i].blobs[j];
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
Eigen::Matrix<Scalar,3,1> triangulatePoint(const std::vector<std::vector<Eigen::Matrix<PointScalar,2,1>> const *> &pointGroups, 
	const std::vector<CameraCalib_t<CalibScalar>> &cameras, TriangulatedPoint &point3D)
{
	Eigen::Matrix<Scalar,3,1> center = Eigen::Matrix<Scalar,3,1>::Zero();
	int centerCnt = 0;
	for (int r = 0; r < pointGroups.size(); r++)
	{
		if (point3D.blobs[r] == InvalidBlob) continue;
		for (int s = r+1; s < pointGroups.size(); s++)
		{
			if (point3D.blobs[s] == InvalidBlob) continue;
			Scalar sec1, sec2;
			Ray3_t<Scalar> ray1 = castRay<Scalar>(pointGroups[r]->at(point3D.blobs[r]), cameras[r]);
			Ray3_t<Scalar> ray2 = castRay<Scalar>(pointGroups[s]->at(point3D.blobs[s]), cameras[s]);
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
Eigen::Matrix<Scalar,3,1> refineTriangulationIterative(const std::vector<std::vector<Eigen::Matrix<PointScalar,2,1>> const *> &pointGroups, 
	const std::vector<CameraCalib_t<CalibScalar>> &cameras, TriangulatedPoint_t<TriScalar> &point3D, int maxIterations, float threshold3D)
{
	typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic> MatrixX;
	typedef Eigen::Matrix<Scalar,4,1> Vector4;
	typedef Eigen::Matrix<Scalar,3,1> Vector3;

	// Get involved cameras
	int camCount = 0;
	for (int c = 0; c < cameras.size(); c++)
		camCount += (point3D.blobs[c] != InvalidBlob);
	// Build base data matrix as well as row-vectors for weights
	MatrixX triSolveBase = MatrixX(camCount*2, 4);
	MatrixX thirdRow = MatrixX(camCount, 4);
	int camIndex = 0;
	for (int c = 0; c < cameras.size(); c++)
	{
		if (point3D.blobs[c] == InvalidBlob) continue;
		auto camMat = cameras[c].camera.matrix().template cast<Scalar>();
		auto point = pointGroups[c]->at(point3D.blobs[c]).template cast<Scalar>();
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
			LOGC(LTrace, "Triangulation got %f error after %d a total of iterations!\n", (triSolve * triResult).cwiseAbs().mean()*1000, i);
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
Eigen::Matrix<Scalar,3,1> refineTriangulation(const std::vector<std::vector<Eigen::Matrix<PointScalar,2,1>> const *> &pointGroups, 
	const std::vector<CameraCalib_t<CalibScalar>> &cameras, TriangulatedPoint &point3D)
{
	return refineTriangulationIterative<Scalar>(pointGroups, cameras, point3D, 0, 0); // Not refining iterations
}

// Generate specific implementations

template Eigen::Vector3f triangulatePoint(const std::vector<std::vector<Eigen::Vector2f> const *> &pointGroups, 
	const std::vector<CameraCalib> &cameras, TriangulatedPoint &point3D);

template Eigen::Vector3f refineTriangulation(const std::vector<std::vector<Eigen::Vector2f> const *> &pointGroups, 
	const std::vector<CameraCalib> &cameras, TriangulatedPoint &point3D);
template Eigen::Vector3d refineTriangulation(const std::vector<std::vector<Eigen::Vector2d> const *> &pointGroups, 
	const std::vector<CameraCalib> &cameras, TriangulatedPoint &point3D);

template Eigen::Vector3f refineTriangulationIterative(const std::vector<std::vector<Eigen::Vector2f> const *> &pointGroups, 
	const std::vector<CameraCalib> &cameras, TriangulatedPoint_t<float> &point3D, int maxIterations, float threshold3D);
template Eigen::Vector3d refineTriangulationIterative(const std::vector<std::vector<Eigen::Vector2d> const *> &pointGroups, 
	const std::vector<CameraCalib> &cameras, TriangulatedPoint_t<float> &point3D, int maxIterations, float threshold3D);
template Eigen::Vector3d refineTriangulationIterative(const std::vector<std::vector<Eigen::Vector2d> const *> &pointGroups, 
	const std::vector<CameraCalib> &cameras, TriangulatedPoint_t<double> &point3D, int maxIterations, float threshold3D);

#ifdef OPT_AUTODIFF
// For AutoDiff used in optimisation/optimisation.hpp ()
typedef Eigen::AutoDiffScalar<VectorX<float>> ActiveScalarf;
typedef Eigen::AutoDiffScalar<VectorX<double>> ActiveScalard;
template Vector3<ActiveScalarf> refineTriangulation(const std::vector<std::vector<Vector2<ActiveScalarf>> const *> &pointGroups, 
	const std::vector<CameraCalib_t<ActiveScalarf>> &cameras, TriangulatedPoint &point3D);
template Vector3<ActiveScalard> refineTriangulation(const std::vector<std::vector<Vector2<ActiveScalard>> const *> &pointGroups, 
	const std::vector<CameraCalib_t<ActiveScalard>> &cameras, TriangulatedPoint &point3D);
template Vector3<ActiveScalarf> refineTriangulationIterative(const std::vector<std::vector<Vector2<ActiveScalarf>> const *> &pointGroups, 
	const std::vector<CameraCalib_t<ActiveScalarf>> &cameras, TriangulatedPoint &point3D, int maxIterations, float threshold3D);
template Vector3<ActiveScalard> refineTriangulationIterative(const std::vector<std::vector<Vector2<ActiveScalard>> const *> &pointGroups, 
	const std::vector<CameraCalib_t<ActiveScalard>> &cameras, TriangulatedPoint &point3D, int maxIterations, float threshold3D);
#endif