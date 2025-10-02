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

#include "simulation.hpp"
#include "pipeline/pipeline.hpp"

#include "util/eigenutil.hpp"

//#define LOG_MAX_LEVEL LTrace
#include "util/log.hpp"

#include <random>
#include <bitset>
#include <algorithm>

/*
 * Generate simulated data
 */


#define MAX_MARKER_POINTS 64

const bool recordPoints = true;

const std::array<MotionParameters, MotionCustom> motionPresets = {
	MotionParameters { 2.0f, 0.0f, 0.00004f, 1.5f, 0.2f, 0.0f, 0.003f, 0.0f, 0.2f },
	MotionParameters { 0.5f, 0.01f, 0.00005f, 1.2f, 0.2f, 0.05f, 0.005f, 0.01f, 0.2f },
	MotionParameters { 0.02f, 0.0015f, 0.00005f, 1.0f, 0.5f, 0.05f, 0.01f, 0.005f, 0.1f }
};


/* Variables */

static std::mt19937 gen = std::mt19937(std::random_device{}());


TargetCalibration3D PointCalibMarker({
	{
		Eigen::Vector3f::Zero(),
		Eigen::Vector3f::UnitZ(),
		330, // Some dead spots to simulate occlusions (360° for full coverage)
		0.04f // 4cm big marker
	}
});

#ifdef USE_LINE_LENGTH
TargetCalibration3D LineCalibMarker({
	{
		Eigen::Vector3f(-30,0,0),
		Eigen::Vector3f(-1,0,0),
		320, // Some dead spots to simulate occlusions (360° for full coverage)
		0.02f // 2cm big marker
	},
	{
		Eigen::Vector3f(+30,0,0),
		Eigen::Vector3f(+1,0,0),
		320, // Some dead spots to simulate occlusions (360° for full coverage)
		0.02f // 2cm big marker
	}
});
#endif


/* Functions */

static void createTargetProjection(std::vector<Eigen::Vector2f> &points2D, std::vector<BlobProperty> &properties, std::vector<int> &markerMap,
	const TargetCalibration3D &target, const CameraCalib &calib,  const CameraMode &mode, const Eigen::Isometry3f &pose, float stdDeviation);

static Eigen::Isometry3f genPoseInTrackingSpace(const std::vector<CameraPipeline> &cameras);

static Eigen::Isometry3f generateFluidPose(const PipelineState &pipeline, SimulatedObject &object);


static Eigen::Isometry3f generateFluidPose(const PipelineState &pipeline, SimulatedObject &object)
{
	auto &motion = object.internalMotionState;
	auto &params = object.motion;
	// Generate consistent movement (positional acceleration is in mm)
	motion.TA += Eigen::Vector3f(
		(rand()%10000 / 10000.0f) * params.accT - params.accT/2,
		(rand()%10000 / 10000.0f) * params.accT - params.accT/2,
		(rand()%10000 / 10000.0f) * params.accT - params.accT/2)/1000;
	motion.RA += Eigen::Vector3f(
		(rand()%10000 / 10000.0f) * params.accR - params.accR/2,
		(rand()%10000 / 10000.0f) * params.accR - params.accR/2,
		(rand()%10000 / 10000.0f) * params.accR - params.accR/2);
	if (motion.TA.norm() < params.minAcc/1000)
		motion.TA = motion.TA.normalized() * params.minAcc/1000;
	// Dampen movement
	motion.TA *= 1.0-params.dampT;
	motion.RA *= 1.0-params.dampR;
	// Correct
	auto attenuate = [](float val, float att){
		att = std::pow(std::abs(val), att);
		return val < 0? -att : att;
	};
	for (const auto &cam : pipeline.cameras)
	{
		const CameraMode &mode = cam->mode;
		const CameraCalib &calib = cam->simulation.calib;
		// Calculate projection of target
		Eigen::Vector3f viewPos = calib.view.cast<float>() * motion.TGT;
		Eigen::Vector2f proj = viewPos.hnormalized();
		// Apply distortion
		Eigen::Vector2f dist = proj*2;
		if (proj.squaredNorm() < 1.0)
			dist = distortPointUnstable(calib, proj, 50);
		// Calculate 2D force vector to keep target in camera view (weaker horizontally)
		Eigen::Vector2f forceVec;
		forceVec.x() = attenuate(dist.x()*mode.factorW, params.centerAttenuation);
		forceVec.y() = attenuate(dist.y()*mode.factorH, params.centerAttenuation);
		// Calculate target position in 3D
		Eigen::Vector2f forceTargetDist = proj - forceVec;
		Eigen::Vector2f forceTargetImg = undistortPoint(calib, forceTargetDist);
		Eigen::Vector3f forceTargetView = viewPos.z() * forceTargetImg.homogeneous();
		Eigen::Vector3f forceTargetWorld = calib.transform.cast<float>() * forceTargetView;
		// Calculate as 3D force vector
		Eigen::Vector3f forceDir = (forceTargetWorld-motion.TGT).normalized();
		// Apply force to correct to center
		motion.TA += forceDir*params.centerForce;
	}
	motion.TA -= motion.TD * params.slowT;
	motion.RA -= motion.RD * params.slowR;
	// Apply
	motion.TGT += motion.TD+motion.TA/2;
	motion.RGT = motion.RGT * getRotationXYZ(motion.RD+motion.RA/2);;
	motion.TD += motion.TA;
	motion.RD += motion.RA;
	// Finalise
	return createModelMatrix(motion.TGT, motion.RGT);
}

/**
 * Generates simulated data according to config and current phase
 */
void GenerateSimulationData(PipelineState &pipeline, FrameRecord &frameState)
{
	ScopedLogCategory optLogCategory(LSimulation);
	auto simLock = pipeline.simulation.contextualLock();
	SimulationState &simulation = *simLock;

	frameState.cameras.resize(pipeline.cameras.size());

	LOGC(LTrace, "Generating points for frame %u:\n", frameState.num);

	int i = -1;
	for (auto &object : simulation.objects)
	{
		i++;
		if (!object.enabled) continue;
		LOGC(LTrace, "    Generating object %d: %s!\n", object.id, object.label.c_str());

		// ----- Generate new pose -----

		Eigen::Isometry3f oldPose = object.pose;
		object.pose = generateFluidPose(pipeline, object);
		//if (object.logPose)
		{
			Eigen::Quaternionf quat(object.internalMotionState.RGT);
			LOGC(LDebug, "Object %s GT Pose: (%.4f, %.4f, %.4f), Quat (%.4f, %.4f, %.4f, %.4f) --"
				"Dynamic state %.3fcm/f, %.3fcm/f^2, %.3f°/f, Change: %.3fcm, %.3f°/f\n",
				object.label.c_str(),
				object.pose.translation().x(), object.pose.translation().y(), object.pose.translation().z(),
				quat.x(), quat.y(), quat.z(), quat.w(),
				object.internalMotionState.TD.norm()*100,
				object.internalMotionState.TA.norm()*100,
				Eigen::AngleAxisf(getQuaternionXYZ(object.internalMotionState.RD)).angle()/PI*180.0f,
				(object.pose.translation() - oldPose.translation()).norm()*100,
				Eigen::AngleAxisf(object.pose.rotation() * oldPose.rotation().transpose()).angle()/PI*180.0f
			);
		}

		// ----- Generate projection -----

		std::vector<int> markerObsCount;
		if (recordPoints)
			markerObsCount.resize(object.target.markers.size());

		for (auto &cam : pipeline.cameras)
		{
			LOGC(LTrace, "  Camera %d Frame %d:\n", cam->id, frameState.num);
			auto &record = frameState.cameras[cam->index];

			// Project marker into camera view (simulated test data)
			int startPts = record.rawPoints2D.size();
			std::vector<int> markerMap;
			createTargetProjection(record.rawPoints2D, record.properties, markerMap, object.target,
				cam->simulation.calib, cam->mode, object.pose, simulation.blobPxStdDev);

			// Keep track of how many times a point is visible

			if (recordPoints)
			{ // Invert GTMarkers2Point to points2GTMarker (merged points are not invertible)
				record.simulation.GTMarkers2Point.insert(record.simulation.GTMarkers2Point.end(),
					markerMap.begin(), markerMap.end());
				record.simulation.points2GTMarker.resize(record.rawPoints2D.size(), -1);
				for (int p = 0; p < markerMap.size(); p++)
				{
					int pt = markerMap[p];
					if (pt < 0) continue;
					markerObsCount[p]++;
					if (record.simulation.points2GTMarker[pt] == -1)
						record.simulation.points2GTMarker[pt] = p;
					else // Merged, multiple markers
						record.simulation.points2GTMarker[pt] = -2;
					LOGC(LTrace, "    Camera %d Point %d maps back to target point %d, recorded %d (-2 == multiple)\n",
						cam->id, pt, p, record.simulation.points2GTMarker[pt]);
				}
				assert(record.simulation.points2GTMarker.size() == record.rawPoints2D.size());
			}
			else
			{ // Shuffle points around
				std::shuffle(record.rawPoints2D.begin()+startPts, record.rawPoints2D.end(), gen);
			}
		}

		// ----- Generate 3D Point Cloud -----

		//if (simulation.recordPoints && i == simulation.primaryObject)
		{ // Recreate ground truth position of points which could have been triangulatedstd::vector<std::vector<Eigen::Vector2f>*> points2D;

			simulation.framePoses.ensureAt(frameState.num) = object.pose;

			LOGC(LTrace, "Recording GT triangulations:\n");

			// Setup interfacing structures (reused for all points)
			std::vector<std::vector<Eigen::Vector2f>> blobContainer(pipeline.cameras.size());
			std::vector<std::vector<Eigen::Vector2f> const *> points2D(pipeline.cameras.size());
			std::vector<CameraCalib> calibs(pipeline.cameras.size());
			for (auto &cam : pipeline.cameras)
			{
				blobContainer[cam->index].resize(1);		 // Used to store a single blob for each camera involved with a point
				points2D[cam->index] = &blobContainer[cam->index]; // Just interfacing
				calibs[cam->index] = cam->calib;
			}

			simulation.triangulatedPoints3D = { frameState.num, object.pose, {} };
			simulation.triangulatedPoints3D.triangulation.reserve(object.target.markers.size());
			for (int i = 0; i < object.target.markers.size(); i++)
			{
				if (markerObsCount[i] < 2)
					continue;
				Eigen::Vector3f gtPoint = object.pose * object.target.markers[i].pos;
				simulation.triangulatedPoints3D.triangulation.emplace_back(i, gtPoint);

				if (SHOULD_LOGC(LTrace))
				{ // Triangulate the point to double check
					TriangulatedPoint triPoint(Eigen::Vector3f::Zero(), 0, 10, pipeline.cameras.size());
					for (int c = 0; c < pipeline.cameras.size(); c++)
						triPoint.blobs[c] = InvalidBlob;
					int traceableCnt = 0;
					for (auto &cam : pipeline.cameras)
					{
						auto &record = frameState.cameras[cam->index];
						for (int j = 0; j < record.simulation.points2GTMarker.size(); j++)
						{
							if (record.simulation.points2GTMarker[j] == i)
							{
								blobContainer[cam->index][0] = undistortPoint(cam->calib, record.rawPoints2D[j]);
								triPoint.blobs[cam->index] = 0;
								traceableCnt++;
							}
						}
					}
					if (traceableCnt >= 2)
					{ // E.g. merged blobs might not be traceable
						Eigen::Vector3f point3D = refineTriangulationIterative<float>(points2D, calibs, triPoint);
						LOGC(LTrace, "  Triangulated target point %d (%.4f, %.4f, %.4f) from observations, error %.2fmm with %f error value\n",
							i, point3D.x(), point3D.y(), point3D.z(), (point3D-gtPoint).norm()*1000, triPoint.error);
					}
					else
					{
						LOGC(LTrace, "  Could not triangulate target point %d from observations since points weren't all traceable (may have been merged)", i);
					}
				}
			}
			LOGC(LTrace, "Entering %d GT triangulations for frame %d\n",
				(int)simulation.triangulatedPoints3D.triangulation.size(), simulation.triangulatedPoints3D.frame);
		}
	}
}

static Eigen::Isometry3f genPoseInTrackingSpace(const std::vector<CameraPipeline> &cameras)
{
	// Min forced on the groundplane
	Eigen::Vector3f min = Eigen::Vector3f::Zero(), max = Eigen::Vector3f::Zero();
	for (int c = 0; c < cameras.size(); c++)
	{
		min = min.array().min(cameras[c].simulation.calib.transform.translation().cast<float>().array());
		max = max.array().max(cameras[c].simulation.calib.transform.translation().cast<float>().array());
	}
	Eigen::Vector3f diff = max-min;
	min += diff*0.2f;
	diff *= 0.6f;
	Eigen::Isometry3f pose;
	pose.translation() = Eigen::Vector3f(
		(rand()%10000 / 10000.0f) * diff.x() + min.x(),
		(rand()%10000 / 10000.0f) * diff.y() + min.y(),
		(rand()%10000 / 10000.0f) * diff.z() + min.z());
	pose.linear() = getRotationZYX(Eigen::Vector3f(
		(rand()%10000 / 10000.0f) * 360 - 180,
		(rand()%10000 / 10000.0f) * 360 - 180,
		(rand()%10000 / 10000.0f) * 360 - 180));
	return pose;
}

/**
 * Projects target into camera view, clipping out-of-view points, merging closeby points, and applying noise
 */
static void createTargetProjection(std::vector<Eigen::Vector2f> &points2D, std::vector<BlobProperty> &properties, std::vector<int> &markerMap,
	const TargetCalibration3D &target, const CameraCalib &calib,  const CameraMode &mode, const Eigen::Isometry3f &pose, float stdDeviation)
{
	// Create MVP in camera space
	Eigen::Isometry3f mv = calib.view.cast<float>() * pose;
	Eigen::Projective3f mvp = calib.camera.cast<float>() * pose;
	// Create random noise generator
	float noiseScaleX = 2.0f/mode.widthPx, noiseScaleY = 2.0f/mode.heightPx;
	std::normal_distribution<float> noise(0, stdDeviation);
	const float maxNoise = 3*stdDeviation;
	// Reserve space for projected points
	points2D.reserve(points2D.size() + target.markers.size());
	properties.reserve(properties.size() + target.markers.size());
	// Determine limit for point size (below which it is to small to be detected)
	float SizeLimit = 2.0/mode.widthPx * 1.0f; // 1px for now
	float mergeFactor = 0.5f; // Unless they are overlapping really bad, the algorithm could potentially discern them still
	// Init marker -> pt map
	markerMap.clear();
	markerMap.resize(target.markers.size(), -1);
	for (int i = 0; i < target.markers.size(); i++)
	{
		const TargetMarker &markerPt = target.markers[i];
		Eigen::Vector3f camPoint = mv * markerPt.pos;
		// Cull back
		if (camPoint.z() < 0)
			continue;
		// Calculate and clip marker points not facing the camera in regards to their field of view
		Eigen::Vector3f ptNrm = mv.linear() * markerPt.nrm;
		float facing = -ptNrm.dot(camPoint.normalized());
		if (facing < markerPt.angleLimit)
			continue;
		// Project point
		Eigen::Vector2f proj = applyProjection2D(calib, camPoint);
		// Clip
		float distSafety = 2.0f;
		if (proj.x() < -mode.sizeW*distSafety || proj.y() < -mode.sizeH*distSafety || proj.x() > mode.sizeW*distSafety || proj.y() > mode.sizeH*distSafety)
			continue;
		// Apply distortion
		Eigen::Vector2f distProj = distortPointUnstable<float>(calib, proj.head<2>(), 1000, 0.001f*PixelSize);
		Eigen::Vector2f check = undistortPoint(calib, distProj);
		float diff = (proj.head<2>()-check).squaredNorm();
		if (diff > 1*1*PixelSize*PixelSize)
			continue; // Unstable
		// Generate noise
		float noiseX = noise(gen), noiseY = noise(gen); // NOTE: Noise select, first unpredictable, second predictable
//		float noiseX = rand()%10000 / 10000.0f * stdDeviation*2, noiseY = rand()%10000 / 10000.0f * stdDeviation*2;
		if (std::abs(noiseX) > maxNoise) noiseX /= std::ceil(std::abs(noiseX)/maxNoise);
		if (std::abs(noiseY) > maxNoise) noiseY /= std::ceil(std::abs(noiseY)/maxNoise);
		Eigen::Vector2f ptPos = distProj + Eigen::Vector2f(noiseX*noiseScaleX, noiseY*noiseScaleY);
		// Clip
		if (ptPos.x() < -mode.sizeW || ptPos.y() < -mode.sizeH || ptPos.x() > mode.sizeW || ptPos.y() > mode.sizeH)
			continue;
		// Determine point size
		//float ptSize = 1.0f + 0.1f/camPoint.z();
		Eigen::Vector3f camUpVec = calib.transform.matrix().col(2).head<3>().cast<float>();
		Eigen::Vector3f sideVec = (markerPt.pos-calib.transform.translation().cast<float>()).cross(camUpVec);
		Eigen::Vector3f sidePos = markerPt.pos + (markerPt.size/2*sideVec.normalized());
		Eigen::Vector2f sideProj = projectPoint2D(mvp, sidePos);
		sideProj = distortPointUnstable<float>(calib, sideProj, 1000, 0.001f*PixelSize);
		float ptSize = (sideProj-distProj).norm()*2;
		//LOGC(LTrace, "CamUpVec %f, sideVec %f, sidePos %f, sideProj %f\n", camUpVec.norm(), sideVec.norm(), sidePos.norm(), sideProj.norm());
		//LOGC(LTrace, "Point size is %.2fpx with %.4fmm source size, f of %f, at distance of %.4fm\n", ptSize*1280/2, markerPt.size*1000, calib.f, (markerPt.pos-calib.transform.translation().cast<float>()).norm());
		if (ptSize < SizeLimit)
			continue;
		int merged = -1;
		for (int p = 0; p < points2D.size(); p++)
		{
			auto &prop = properties[p];
			float distSq = (points2D[p]-ptPos).squaredNorm();
			if (distSq < (prop.size+ptSize)*(prop.size+ptSize) * mergeFactor*mergeFactor)
			{ // Both sides need to be /4 (/2 + sqrt) - 2D point dist because its -1 to 1, and size because we need radius
				LOGC(LDebug, "Merged with point %d at distance %.2fpx with size %.2fpx and %.2fpx\n",
					p, std::sqrt(distSq)*1280/2, prop.size*1280/2, ptSize*1280/2);
				points2D[p] = (prop.size*points2D[p] + ptSize*ptPos) / (prop.size + ptSize);
				LOGC(LTrace, "    While projecting point %d (%f, %f), decided to merge into point %d instead. "
					"Distance %f, sizes %f and %f, new size %f, pos  (%f, %f)\n",
						(int)points2D.size(), ptPos.x()*PixelFactor, ptPos.y()*PixelFactor, p, std::sqrt(distSq)*PixelFactor, ptSize*PixelFactor,
						prop.size*PixelFactor, (prop.size/2 + ptSize/2 + std::sqrt(distSq))*PixelFactor, points2D[p].x()*PixelFactor, points2D[p].y()*PixelFactor);
				prop.size = prop.size/2 + ptSize/2 + std::sqrt(distSq);
				merged = p;
				break;
			}
			else if (distSq < (prop.size+ptSize)*2*(prop.size+ptSize)*2)
			{
				LOGC(LTrace, "    While projecting point %d (%f, %f), nearly merged into point %d instead. Distance %f, sizes %f and %f\n",
					(int)points2D.size(), ptPos.x()*PixelFactor, ptPos.y()*PixelFactor, p, std::sqrt(distSq)*PixelFactor, ptSize*PixelFactor, properties[p].size*PixelFactor);
			}
		}
		if (merged >= 0)
		{
			markerMap[i] = merged;
			LOGC(LTrace, "    Camera %d: Point merged into %d!", calib.id, merged);
			continue;
		}
		else
		{
			// Register projected marker point
			markerMap[i] = points2D.size();
			points2D.push_back(ptPos);
			BlobProperty prop;
			prop.size = ptSize;
			prop.value = 1000;
			properties.push_back(prop);
			LOGC(LTrace, "    Camera %d: Done projecting point %d (%f, %f), size %f\n", calib.id, (int)points2D.size(), ptPos.x()*PixelFactor, ptPos.y()*PixelFactor, ptSize*PixelFactor);
		}
	}
}