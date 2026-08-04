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

typedef int8_t RayIxCnt; // Limits number of intersections per ray. Theoretically unlimited.
const BlobIndex InvalidBlob = (BlobIndex)-1;

struct MergedIntersection
{
	Eigen::Vector3f center;
	float error;
	int merged;
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


float getTriConfidence(int obsClean, int obsConflicted)
{
	//return (float)(obsClean*obsClean)/(obsConflicted+1);
	return obsClean*obsClean*2 + obsConflicted;
}

float calculate2DSizeSimple(const CameraCalib &calib, Eigen::Vector3f pos, float size3D)
{
	return size3D * calib.f / (pos - calib.transform.translation().cast<float>()).norm();
}

float estimate3DSizeSimple(const CameraCalib &calib, Eigen::Vector3f pos, float size2D)
{
	return size2D * calib.fInv * (pos - calib.transform.translation().cast<float>()).norm();
}

float estimate3DSize(const CameraCalib &calib, Eigen::Vector3f pos, Eigen::Vector2f raw2D, float size2D)
{
	// TODO: Even this 2D ellipse size handling is not entirely correct
	// It doesn't account for the exact elliptical area
	// It just takes the size perpendicular to the image center (which is usually the smallest)
	// So it SHOULD overestimate size
	// But it actually tends to underestimate still (perhaps due to flare in image), so whatever
	// TODO: Let camera handle blob size with full access to blob shape and intrinsic calibration
	Eigen::Vector2f offset = raw2D.normalized() * size2D;
	Eigen::Vector2f ptF = undistortPoint<float>(calib, raw2D + offset);
	Eigen::Vector2f ptN = undistortPoint<float>(calib, raw2D - offset);
	Ray3f rayF = castRay<float>(ptF, calib);
	Ray3f rayN = castRay<float>(ptN, calib);
	float dist = (pos - calib.transform.translation().cast<float>()).norm();
	return (rayF.dir - rayN.dir).norm() * dist / 2;
}

static void findInitialRayIntersections(const std::vector<CameraCalib> &cameras,
	const std::vector<std::vector<Eigen::Vector2f> const *> &points2D, const std::vector<std::vector<int>> &relevantPoints2D,
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
		for (int p : relevantPoints2D[c])
		{
			rayGroups[c][p] = castRay<float>(points2D[c]->at(p), cameras[c]);
		}
	}

	// Fill with candidate intersections
	for (int i = 0; i < camCount-1; i++)
	{
		for (int j = i+1; j < camCount; j++)
		{
			for (BlobIndex v : relevantPoints2D[i])
			{
//				const Ray3f ray1 = castRay<float>(points1->at(v), cameras[i]);
				const Ray3f ray1 = rayGroups[i][v];
				for (BlobIndex w : relevantPoints2D[j])
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
	LOGC(LDebug, "Triangulations handles %d 2-intersections!\n", (int)intersections.size());
}

void triangulateRayIntersections(const std::vector<CameraCalib> &cameras, 
	const std::vector<std::vector<Eigen::Vector2f> const *> &points2D, const std::vector<std::vector<int>> &relevantPoints2D,
	std::vector<TriangulatedPoint> &points3D, float maxError, float minError)
{
	ScopedLogCategory scopedLogCategory(LTriangulation);

	#define IXNUM(ix) (int)(ix == NULL? -1 : (((intptr_t)ix-(intptr_t)intersections.data())/(sizeof(TwoIntersection))))

	// NOTE: This is the subset camera count, also used for indexing throughout (may be changed manually for records)
	int camCount = cameras.size();

	float t_test, t_close = 0, t_merge = 0, t_post, t_total;
	TimePoint_t t0 = sclock::now();

	// Find initial set of 2-intersections
	thread_local std::vector<TwoIntersection> intersections;
	intersections.clear();
	findInitialRayIntersections(cameras, points2D, relevantPoints2D, intersections, rayIxCnt, maxError, minError);
	int ixCnt = intersections.size();

	t_test = dtMS(t0, sclock::now());

	// Have to reserve to prevent reallocation, since it relies on pointers to merged intersections
	thread_local std::vector<MergedIntersection> mergedIntersections;
	mergedIntersections.clear();
	mergedIntersections.reserve(ixCnt/2+1);

	// Prepare allocation for intersection buffers
	thread_local std::vector<TwoIntersection*> mergers, potentialMergers;
	mergers.reserve((camCount+1)*camCount/2);
	potentialMergers.reserve(mergers.size() * 2);

	// Prepare intermediary buffer for blobs used for each camera
	thread_local std::vector<BlobIndex> ixBlobs;
	ixBlobs.clear();
	ixBlobs.resize(camCount, InvalidBlob);

	// Merge possible intersections
	for (int i = 0; i < intersections.size(); i++)
	{
		TwoIntersection *ix = &intersections[i];

		if (ix->merge != NULL)
		{ // Already merged into a pior two-intersection
			continue;
		}

		if (rayIxCnt[ix->c1][ix->b1] <= 1 || rayIxCnt[ix->c2][ix->b2] <= 1)
		{ // If any involved ray has no other intersection, it cannot possibly be merged
			continue;
		}

		LOGC(LTrace, "------ Intersection %d on cameras %d and %d", i, ix->c1, ix->c2);
		ixBlobs[ix->c1] = ix->b1;
		ixBlobs[ix->c2] = ix->b2;

		// Search for other intersections with one common ray and one new ray
		auto testMerger = [&](TwoIntersection* &ixm, int testCam)
		{
			float errorSq = (ixm->center - ix->center).squaredNorm()/4;
			float distSq = (cameras[testCam].transform.translation().cast<float>() - ix->center).squaredNorm();
			float errorConeSq = maxError*maxError * distSq * (float)cameras[testCam].f*(float)cameras[testCam].f;
			if (errorSq > errorConeSq) return;
			// Merge intersections, now consisting of three rays intersecting
			ixBlobs[ixm->c1] = ixm->b1;
			ixBlobs[ixm->c2] = ixm->b2;
			mergers.push_back(ixm);
			ixm = nullptr;
		};

		// Find all close intersections
		// TODO: Use nanoflann? Not the most critical bottleneck right now
		TimePoint_t t10 = sclock::now();
		float maxRange = maxError*2 * 32;
		potentialMergers.clear();
		for (int j = i+1; j < intersections.size(); j++)
		{
			TwoIntersection *ixm = &intersections[j];
			if (ixm->merge != NULL) continue;
			if ((ixm->center - ix->center).squaredNorm() < maxRange*maxRange)
			{ // Goal is to handle ANY that could possibly be merged, to limit cost of iterations
				potentialMergers.push_back(ixm);
			}
		}
		t_close += dtMS(t10, sclock::now());

		TimePoint_t t11 = sclock::now();
		int count = -1;
		mergers.clear();
		int rem = potentialMergers.size();
		while (count != mergers.size() && rem > 0)
		{
			count = mergers.size();
			rem = 0;
			for (auto &ixm : potentialMergers)
			{
				if (!ixm) continue;
				if (ixBlobs[ixm->c1] == ixm->b1)
				{
					if (ixBlobs[ixm->c2] == InvalidBlob)
						testMerger(ixm, ixm->c1 == ix->c2? ix->c1 : ix->c2);
					else if (ixBlobs[ixm->c2] == ixm->b2)
					{
						mergers.push_back(ixm);
						ixm = nullptr;
					}
					// else // Technically, a conflict here COULD be better matching than an intersection already included...
				}
				else if (ixBlobs[ixm->c2] == ixm->b2)
				{
					if (ixBlobs[ixm->c1] == InvalidBlob)
						testMerger(ixm, ixm->c2 == ix->c2? ix->c1 : ix->c2);
					else if (ixBlobs[ixm->c1] == ixm->b1)
					{
						mergers.push_back(ixm);
						ixm = nullptr;
					}
					// else // Technically, a conflict here COULD be better matching than an intersection already included...
				}
				if (ixm) rem++;
			}
		}
		t_merge += dtMS(t11, sclock::now());

		// Check if merge candidates found
		if (mergers.empty())
		{ // Clean up
			ixBlobs[ix->c1] = InvalidBlob;
			ixBlobs[ix->c2] = InvalidBlob;
			continue;
		}

		// Add original intersection
		mergers.push_back(ix);

		LOGC(LTrace, "    Merged %d / %d closeby 2-intersections!\n", (int)mergers.size()-1, (int)potentialMergers.size());

		if (SHOULD_LOGC(LTrace))
		{
			std::string blobsStr = "";
			for (int j = 0; j < camCount; j++)
				blobsStr += ixBlobs[j] == InvalidBlob? "X - " : asprintf_s("%d - ", ixBlobs[j]);
			LOGC(LTrace, "    Merging Blobs: %s", blobsStr.c_str());
		}

		if (SHOULD_LOGC(LTrace))
		{
			std::string blobsStr = "";
			for (auto &ixm : mergers)
				blobsStr += asprintf_s("%d, ", IXNUM(ixm));
			LOGC(LTrace, "    Merging Intersections: %s", blobsStr.c_str());
		}

		// Update intersection metrics rayIxCnt and ixCnt
		for (auto &ixm : mergers)
		{
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
		MergedIntersection &mergedIx = mergedIntersections.back();
		for (auto &ixm : mergers)
		{
			ixm->merge = &mergedIx;
			mergedIx.center += ixm->center;
			mergedIx.error += ixm->error;
			// NOTE: This is not the true center of the merged intersection, just a quick approximation
			// refineTriangulation/refineTriangulationIterative are used later to improve it
		}
		mergedIx.center = mergedIx.center / mergers.size();
		mergedIx.error = mergedIx.error / mergers.size();
		mergedIx.merged = mergers.size();
		mergedIx.blobs = std::move(ixBlobs);

		// Prepare ixBlobs for next iteration
		ixBlobs.clear();
		ixBlobs.resize(camCount, InvalidBlob);
	}

	// Compile all intersections as triangulated points
	TimePoint_t t2 = sclock::now();
	points3D.reserve(points3D.size() + ixCnt);
	auto handlePoints = [&](auto &ixm)
	{
		int clean = 0, conflict = 0;
		std::vector<TriangulatedPoint::TriSample> samples;
		auto blob = [&](int c, int b)
		{
			assert(points2D[c]->size() > b);
			if (rayIxCnt[c][b] == 1) clean++;
			else conflict++;
			samples.emplace_back(c, b);
		};
		if constexpr (std::is_same_v<decltype(ixm), MergedIntersection&>)
		{ // MergedIntersection
			samples.reserve(ixm.merged);
			for (int c = 0; c < camCount; c++)
				if (ixm.blobs[c] != InvalidBlob)
					blob(c, ixm.blobs[c]);
		}
		else
		{ // TwoIntersection
			samples.reserve(2);
			blob(ixm.c1, ixm.b1);
			blob(ixm.c2, ixm.b2);
		}
		float confidence = getTriConfidence(clean, conflict);
		points3D.emplace_back(ixm.center, ixm.error, confidence);
		points3D.back().samples = std::move(samples);
	};
	for (auto &ixm : mergedIntersections)
	{
		handlePoints(ixm);
	}
	for (auto &ix : intersections)
	{
		if (ix.merge == NULL)
			handlePoints(ix);
	}
	assert(points3D.size() == ixCnt);
	t_post = dtMS(t2, sclock::now());

	t_total = dtMS(t0, sclock::now());
	LOG(LTriangulation, LDebug, "Triangulation took %.2fms: %.3fms - [ %.3fms - %.3fms ] - %.3fms", t_total, t_test, t_close, t_merge, t_post);
}

void resolveTriangulationConflicts(const std::vector<CameraCalib> &cameras, std::vector<TriangulatedPoint> &points3D, float maxError, float confidenceThreshold)
{
	ScopedLogCategory scopedLogCategory(LTriangulation);

	// Sort by confidence and, secondarily, error (can also be used to punish high errors more severely)
	std::sort(points3D.begin(), points3D.end(), [maxError](const TriangulatedPoint &a, const TriangulatedPoint &b){ 
		return (a.confidence-a.error/maxError) > (b.confidence-b.error/maxError);
	});
	// could get away with no sorting by doing an additional pass over all points, but sorting speeds stuff up down the line anyways

	int index = 0;
	for (auto &tri : points3D)
	{
		int clean = 0, conflict = 0;
		for (auto &sample : tri.samples)
		{
			if (rayIxCnt[sample.camera][sample.blob] < 0)
			{ // Already claimed by a point with higher confidence
				conflict++;
			}
			else
			{ // Else claim it (doesn't matter if there's only one intersection on this ray)
				assert(rayIxCnt[sample.camera][sample.blob] > 0);
				rayIxCnt[sample.camera][sample.blob] = -rayIxCnt[sample.camera][sample.blob];
				clean++;
			}
		}
		// Calculate new confidence:
		tri.confidence = getTriConfidence(clean, conflict);
		LOGC(LTrace, "    Point %d: Error: %f, Initial Confidence: %f, nc=%d, c=%d\n", index++, tri.error, tri.confidence, clean, conflict);

		if (tri.confidence < confidenceThreshold)
		{
			for (auto &sample : tri.samples)
			{ // Remove from conflicts (it's negative as it has been claimed)
				rayIxCnt[sample.camera][sample.blob]++;
				assert(rayIxCnt[sample.camera][sample.blob] <= 0);
			}
		}
	}
	index = 0;
	for (int i = 0; i < points3D.size(); i++)
	{
		auto &tri = points3D[i];
		if (tri.confidence < confidenceThreshold)
		{
			LOGC(LTrace, "    Dropped point %d in first iteration!\n", i);
			continue;
		}
		int clean = 0, conflict = 0;
		for (int s = 0, ss = 0; s < tri.samples.size(); s++)
		{
			auto &sample = tri.samples[s];
			if (rayIxCnt[sample.camera][sample.blob] == -1)
			{ // No conflict, just this intersections claiming this blob
				clean++;
				tri.samples[ss++] = tri.samples[s];
				continue;
			}
			// Another intersection claimed it and prevailed
			// This ray is likely a merged blob, drop it
			// TODO: Consider more sophisticated selection method? Blob later down the ray may be fully occluded
			// Just risky to assume anything based on just observed error
			assert(rayIxCnt[sample.camera][sample.blob] < -1);
			conflict++; // Since this DOES still help the confidence
		}
		// Calculate new confidence (may be slightly higher if a conflict got resolved cleanly):
		tri.confidence = getTriConfidence(clean, conflict);

		if (clean >= 2 && tri.confidence >= confidenceThreshold)
		{ // Can triangulate and is confident, keep
			if (i != index)
				std::swap(points3D[index], points3D[i]);
			index++;
			LOGC(LTrace, "    Point %d: Error: %f, Confidence: %f, nc=%d, c=%d\n", i, tri.error, tri.confidence, clean, conflict);
		}
		else
			LOGC(LTrace, "    Dropped point %d! Confidence: %f, nc=%d, c=%d\n", i, tri.confidence, clean, conflict);
	}
	points3D.resize(index);
}

/**
 * Refine triangulation accuracy of point by minimising the reprojection error iteratively (nearly projection invariant)
 * NOTE: Relies on TriangulatedPoint::TriSample::camera indexing into given subset of cameras
 */
template<typename Scalar, typename PointScalar, typename CalibScalar, typename TriScalar>
Eigen::Matrix<Scalar,3,1> refineTriangulationIterative(const std::vector<std::vector<Eigen::Matrix<PointScalar,2,1>> const *> &points2D, 
	const std::vector<CameraCalib_t<CalibScalar>> &cameras, TriangulatedPoint_t<TriScalar> &point3D, int maxIterations, float threshold3D)
{
	typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic> MatrixX;
	typedef Eigen::Matrix<Scalar,4,1> Vector4;
	typedef Eigen::Matrix<Scalar,3,1> Vector3;

	// Build base data matrix as well as row-vectors for weights
	int camCount = point3D.samples.size();
	MatrixX triSolveBase = MatrixX(camCount*2, 4);
	MatrixX thirdRow = MatrixX(camCount, 4);
	int camIndex = 0;
	for (auto &sample : point3D.samples)
	{
		auto camMat = cameras[sample.camera].camera.matrix().template cast<Scalar>();
		auto point = points2D[sample.camera]->at(sample.blob).template cast<Scalar>();
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
 * NOTE: Relies on TriangulatedPoint::TriSample::camera indexing into given subset of cameras
 */
template<typename Scalar, typename PointScalar, typename CalibScalar>
Eigen::Matrix<Scalar,3,1> refineTriangulation(const std::vector<std::vector<Eigen::Matrix<PointScalar,2,1>> const *> &points2D, 
	const std::vector<CameraCalib_t<CalibScalar>> &cameras, TriangulatedPoint &point3D)
{
	return refineTriangulationIterative<Scalar>(points2D, cameras, point3D, 0, 0); // Not refining iterations
}

// Generate specific implementations

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