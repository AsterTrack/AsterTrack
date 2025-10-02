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

#include "reconstruction.hpp"

#include "point/triangulation.hpp"

#include "tensor/tucker_decomposition.hpp"

#include "util/log.hpp"
#include "util/eigenalg.hpp"

#include <set>

// Used for development
const bool useGTdata = false, useGTsolution = false;

// Don't replace with newly estimated projective depth because it's unstable, instead lerp to it
const float projDepthCorrectionRate = 0.1f;

/**
 * Accumulate all observations of the target in a data tensor [cameras, frames, markers]
 * Triangulates all triangulatable observations and puts them with their projective depth in an immutable section
 * Approximates projective depth of all other observations using those triangulations and puts them in the mutable section
 */
static std::tuple<int,int,int,int> buildDataTensor(const ObsTarget &target, const std::vector<CameraCalib> &cameraCalibs, SparseTensor<double, 3> &dataTensor, Eigen::VectorXd &measurements, Eigen::VectorXd &projectiveDepths);

/**
 * The algorithm is super unstable right now, so we generate a bunch and pick the "best" candidate
 * Instability means some frames are often not properly reconstructed, but usually do have proper reconstructions sometimes
 * But then, some others would randomly fail to reconstruct - probably the same for the markers, not sure
 */
struct SolutionCandidate
{
    double error, markerErrorGT;
	Eigen::VectorXi disableFrame, disableMarker;
	Eigen::MatrixXd motion, structure;
};

/**
 * Attempts to reconstruct the motion and structure of a single rigid target given its observations
 */
[[gnu::flatten, gnu::target_clones("arch=x86-64-v4", "default")]]
bool reconstructTarget(const std::vector<CameraCalib> &cameraCalibs, ObsTarget &target,
	int pMaxIteration, int pMinAbortIteration, int pCorrectIteration, int pDropIteration,
	ObsTarget *targetGT, std::vector<float> *frameChanges, std::vector<float> *frameErrors, std::vector<float> *frameGTDiff)
{
	ScopedLogCategory scopedLogCategory(LTargetReconstruction);

	// TODO: Target reconstruction does not work as an optimiser as expected, just a randomized structure generator
	// Each iteration should improve the error quickly, but it doesn't
	// Instead, currently, the best candidate out of all "iterations" is used for later traditional optimisation
	// It's slower, but it seems to work, so this is left as-is for now
	// Really this is just an initialisation for regular optimisation anyway at this point, and it does that job ok

	// ----- Data aquisition

	// TODO: Only regard cameras that observed relevant markers
	int markerCount = target.markers.size();
	int camCount = cameraCalibs.size();
	int frameCount = target.frames.size();
	int obsCount = target.totalSamples;
	if (markerCount == 0 || camCount == 0 || frameCount == 0)
		return false;
	int frameStart = target.frames.front().frame;
	int frameEnd = target.frames.back().frame+1;
	if (frameChanges)
		frameChanges->resize(frameEnd-frameStart);
	if (frameErrors)
		frameErrors->resize(frameEnd-frameStart);
	if (frameGTDiff)
		frameGTDiff->resize(frameEnd-frameStart);

	// Sparse data tensor filled with point correspondences used for factorisation, split into 2D and 3D sections
	SparseTensor<double, 3> dataTensor;
	// Used to iteratively correct the projectiveDepths in the 2D section of the data tensor
	Eigen::VectorXd measurements, projectiveDepths;
	auto s = buildDataTensor(target, cameraCalibs, dataTensor, measurements, projectiveDepths);
	int triPointCount = std::get<0>(s), triSourceCount = std::get<1>(s), mutableCount = std::get<2>(s), immutableCount = std::get<3>(s);

	LOGC(LDebug, "Data Tensor has %d out of %d entries (%.2f%% of total)\n", (int)dataTensor.entries(), (int)dataTensor.size(), ((float)dataTensor.entries()/dataTensor.size())*100);

	// GT equivalents
	SparseTensor<double, 3> dataTensorGT;
	Eigen::VectorXd measurementsGT, projectiveDepthsGT, projectiveDepthsEstGT;
	if (targetGT)
	{
		buildDataTensor(*targetGT, cameraCalibs, dataTensorGT, measurementsGT, projectiveDepthsEstGT);

		// Recreate exact GT projective depth
		projectiveDepthsGT = projectiveDepthsEstGT;
		for (int i = 0; i < mutableCount; i += 2)
		{
			int dc = dataTensorGT.indices(i,0), f = dataTensorGT.indices(i,1), s = dataTensorGT.indices(i,2), c = dc/3;
			Eigen::Vector3f gtPoint = targetGT->frames[f].pose * targetGT->markers[s];
			projectiveDepthsGT(i/2) = (cameraCalibs[c].camera.cast<float>() * gtPoint.homogeneous()).w();
		}
	}

	int dataCount = mutableCount+immutableCount;
	int recFrameCount = frameCount;

	LOGC(LDebug, "Got %d 2D points plus %d 3D points, so a mutable ratio of %f%% in %d total data points\n",
		obsCount - triSourceCount, triPointCount, (float)mutableCount/dataCount*100, dataCount);

	typedef IterativeSparseTuckerDecomposition<double, 3, 13> Decomp;


	Eigen::VectorXi disableFrame = Eigen::VectorXi::Zero(frameCount);
	// TODO: Allow disabling markers if they are not part of the target (currently not used)
	Eigen::VectorXi disableMarker = Eigen::VectorXi::Zero(markerCount);

	std::vector<SolutionCandidate> solutionCandidates;
	auto recordSolution = [&](const Decomp &tucker, const ObsTarget &target, const Eigen::VectorXd &frameErr, const Eigen::VectorXi &frameNum)
	{
		Eigen::Isometry3d lastPose, lastStablePose;
		std::set<int> lastStableMarkers;
		int lastFrame = target.frames.front().frame;

		const Eigen::MatrixXd &motion = tucker.factor<1>();
		const Eigen::MatrixXd &structure = tucker.factor<2>();

		double ratingError = 0, ratingGT = 0;
		int frameCnt = 0;

		solutionCandidates.push_back({});
		SolutionCandidate &cand = solutionCandidates.back();
		cand.motion = motion;
		cand.structure = structure;
		cand.disableFrame = disableFrame;
		cand.disableMarker = disableMarker;

		for (int f = 0; f < motion.rows(); f++)
		{
			if (disableFrame(f)) continue;

			frameCnt++;
			ratingError += frameErr(f)*frameErr(f);

			int frame = target.frames[f].frame;
			Eigen::Vector<double,13> transform = motion.row(f);
			Eigen::Isometry3d pose;
			pose.linear() = Eigen::Map<Eigen::Matrix3d>(transform.data());
			pose.translation() = Eigen::Map<Eigen::Vector3d>(transform.data()+9);

			if (frameErrors)
			{
				for (int i = lastFrame+1; i < frame; i++)
					frameErrors->at(i-frameStart) = 0;
				frameErrors->at(frame-frameStart) = (float)frameErr(f);
			}

			if (!targetGT) continue;

			float diffGTMarkers = 0;
			for (int m = 0; m < markerCount; m++)
			{
				Eigen::Vector4f markerGT = targetGT->markers[m].homogeneous();
				Eigen::Vector4f projGT = targetGT->frames[f].pose * markerGT;
				Eigen::Vector4d marker = tucker.factor<2>().row(m);
				Eigen::Vector4d projSim = pose * marker;
				diffGTMarkers += (projSim.hnormalized().cast<float>()-projGT.hnormalized()).squaredNorm();
			}
			ratingGT += std::sqrt(diffGTMarkers/markerCount);

			if (frameGTDiff)
			{
				Eigen::Isometry3d diffGT = pose * targetGT->frames[f].pose.inverse().cast<double>();
				double diffGTVal = diffGT.translation().norm()*1000 + Eigen::AngleAxisd(diffGT.rotation()).angle()*180/PI;
				for (int i = lastFrame+1; i < frame; i++)
					frameGTDiff->at(i-frameStart) = 0;
				frameGTDiff->at(frame-frameStart) = (float)diffGTVal/10;
			}
		}

		cand.markerErrorGT = ratingGT/frameCnt*1000; // marker rmse to mm
		cand.error = std::sqrt(ratingError/frameCnt)*PixelFactor; // Error in pixels

		LOGC(LDebug, "        Sample rating: %.4fpx error, %.4fmm GT Marker RMSE\n", cand.error, cand.markerErrorGT);
	};
	auto debugSolutionByFrames = [&](const Decomp &tucker, const ObsTarget &target, const Eigen::VectorXd &frameErr, const Eigen::VectorXi &frameNum)
	{
		Eigen::Isometry3d lastPose, lastStablePose;
		std::set<int> lastStableMarkers;
		int lastFrame = target.frames.front().frame;

		const Eigen::MatrixXd &motion = tucker.factor<1>();
		const Eigen::MatrixXd &structure = tucker.factor<2>();

		double lastDiff = 0, diffStable = 1;
		double lastStableDiff, diffTransition = 0;
		int inTransition = 0;
		double prevStableError = 0;
		int prevStableLength = 0;

		double ratingError = 0, ratingChange = 0, ratingCombined = 0;
		double ratingGTPos = 0, ratingGTRot = 0, ratingGT = 0;
		int frameCnt = 0;

		solutionCandidates.push_back({});
		SolutionCandidate &cand = solutionCandidates.back();
		cand.motion = motion;
		cand.structure = structure;
		cand.disableFrame = disableFrame;

		for (int f = 0; f < motion.rows(); f++)
		{
			if (disableFrame(f)) continue;

			frameCnt++;
			ratingError += frameErr(f)*frameErr(f);

			int frame = target.frames[f].frame;
			Eigen::Vector<double,13> transform = motion.row(f);
			Eigen::Isometry3d pose;
			pose.linear() = Eigen::Map<Eigen::Matrix3d>(transform.data());
			pose.translation() = Eigen::Map<Eigen::Vector3d>(transform.data()+9);

			if (frameErrors)
			{
				for (int i = lastFrame+1; i < frame; i++)
					frameErrors->at(i-frameStart) = 0;
				frameErrors->at(frame-frameStart) = frameErr(f)*PixelFactor;
			}

			if (targetGT)
			{
				Eigen::Isometry3f poseGT = targetGT->frames[f].pose;
				float diffGTMarkers = 0;
				for (int m = 0; m < markerCount; m++)
				{
					Eigen::Vector4f markerGT = targetGT->markers[m].homogeneous();
					Eigen::Vector4f projGT = targetGT->frames[f].pose * markerGT;
					Eigen::Vector4d marker = tucker.factor<2>().row(m);
					Eigen::Vector4d projSim = pose * marker;
					diffGTMarkers += (projSim.hnormalized().cast<float>()-projGT.hnormalized()).squaredNorm();
				}
				diffGTMarkers = std::sqrt(diffGTMarkers/markerCount);

				Eigen::Isometry3d diffGT = pose * targetGT->frames[f].pose.inverse().cast<double>();
				double diffGTVal = diffGT.translation().norm()*1000 + Eigen::AngleAxisd(diffGT.rotation()).angle()*180/PI;
				ratingGT += diffGTMarkers;
				ratingGTPos += diffGT.translation().norm()*1000;
				ratingGTRot += Eigen::AngleAxisd(diffGT.rotation()).angle()*180/PI;

				if (frameGTDiff)
				{
					for (int i = lastFrame+1; i < frame; i++)
						frameGTDiff->at(i-frameStart) = 0;
					frameGTDiff->at(frame-frameStart) = (float)diffGTVal/10;
				}
			}

			if (f == 0)
			{
				lastPose = pose;
				lastFrame = frame;
				continue;
			}

			Eigen::Isometry3d diff = pose * lastPose.inverse();
			double diffVal = diff.translation().norm()*1000 + Eigen::AngleAxisd(diff.rotation()).angle()/PI*180;
			double avgChange = diffVal/(frame-lastFrame);
			ratingChange += avgChange*avgChange;
			if (frameChanges)
			{
				for (int i = lastFrame; i < frame; i++)
					frameChanges->at(i-frameStart) = (float)avgChange;
			}

			if (inTransition > 0)
			{
				//LOGC(LTrace, "Frame %d: In HIGH state with value %f (HIGH limit %f)\n", frame, diffVal, diffTransition/3);
				if (diffVal > diffTransition/3 && diffVal < diffStable*5)
				{
					diffTransition += (diffVal-diffTransition)/3;
					//LOGC(LTrace, "Frame %d in another high, %d frames in: Diff before %f - now %f\n", frame, inTransition, lastDiff, diffVal);
					inTransition++;
				}
				else
				{ // Leaving transition phase
					Eigen::Isometry3d stableDiff = pose * lastStablePose.inverse();
					double stableDiffVal = stableDiff.translation().norm()*1000 + Eigen::AngleAxisd(stableDiff.rotation()).angle()/PI*180;
					int sharedObsCnt = 0;
					for (auto &sample : target.frames[f].samples)
						if (lastStableMarkers.find(sample.marker) != lastStableMarkers.end())
							sharedObsCnt++;
					LOGC(LTrace, "        Frame %d transition for %d frames after stable period of %d frames, avg error of %fpx. Diff to last stable %f, shared sample count %d\n",
						frame, inTransition, prevStableLength, prevStableError/prevStableLength*PixelFactor, stableDiffVal, sharedObsCnt);
					//LOGC(LTrace, "Frame %d in high for %d frames: Diff before %f - floating %f - after %f\n",
					//	frame, inTransition, lastStableDiff, diffTransition, diffVal);
					//LOGC(LTrace, "Frame %d: Entered LOW state with value %f (low limit now %f)\n", fra<me, diffVal, diffVal/2);
					diffStable = diffVal/2;
					inTransition = 0;
					prevStableError = frameErr(f);
					prevStableLength = 1;
				}
			}
			else
			{
				if (diffVal < diffStable*5)
				{ // Frame diff ok
					diffStable += (diffVal-diffStable)/5;
					diffStable = std::max(diffStable, 5.0); // 5mm or 5 degrees
					prevStableError += frameErr(f);
					prevStableLength++;
				}
				else
				{ // Frame diff entered high "transition" state
					//LOGC(LTrace, "Frame %d: Entered HIGH state from %f to %f (limit %f)\n", frame, lastDiff, diffVal, diffStable*3);
					lastStableDiff = lastDiff;
					lastStablePose = lastPose;
					lastStableMarkers.clear();
					for (auto &sample : target.frames[f].samples)
						lastStableMarkers.insert(sample.marker);
					inTransition = 1;
					diffTransition = diffVal;
				}
			}
			lastDiff = diffVal;
			lastPose = pose;
			lastFrame = frame;
		}

		ratingError = std::sqrt(ratingError/frameCnt)*PixelFactor; // Change in pixels
		ratingChange = std::sqrt(ratingChange/frameCnt); // Change in mm+dg
		ratingCombined = ratingError + ratingChange;

		ratingGTPos = ratingGTPos/frameCnt;
		ratingGTRot = ratingGTRot/frameCnt;
		ratingGT = ratingGT/frameCnt*1000; // marker rmse to mm

		LOGC(LDebug, "        Sample rating: %.4f error, %.4f change, %.4f rating, %.4fmm GT Marker RMSE, GT Pose: %.4fmm, %.4f°\n",
			ratingError, ratingChange, ratingCombined, ratingGT, ratingGTPos, ratingGTRot);

		cand.markerErrorGT = ratingGT;
		cand.error = ratingError;

	};

	auto correctProjectiveDepth = [mutableCount, &disableFrame](const Decomp &tucker, SparseTensor<double, 3> &dataTensor, const Eigen::VectorXd &measurements, Eigen::VectorXd &projDepths)
	{
		for (int i = 0; i < mutableCount; i += 2)
		{
			int dc = dataTensor.indices(i,0), f = dataTensor.indices(i,1), s = dataTensor.indices(i,2), c = dc/3;
			if (disableFrame(f)) continue;
			projDepths(i/2) += (tucker(Eigen::Vector3i(dc+2, f, s))-projDepths(i/2)) * projDepthCorrectionRate;
			dataTensor.values(i+0) = measurements(i+0)*projDepths(i/2);
			dataTensor.values(i+1) = measurements(i+1)*projDepths(i/2);
		}
	};
	auto getProjectiveDepths = [mutableCount](const Decomp &tucker, const SparseTensor<double, 3> &dataTensor, Eigen::VectorXd &projDepths)
	{
		for (int i = 0; i < mutableCount; i += 2)
		{
			int dc = dataTensor.indices(i,0), f = dataTensor.indices(i,1), s = dataTensor.indices(i,2);
			projDepths(i/2) = tucker(Eigen::Vector3i(dc+2, f, s));
		}
	};

	auto calculateErrors = [mutableCount, immutableCount, &disableFrame](const char *label, const Decomp &tucker, const SparseTensor<double, 3> &dataTensor, const Eigen::VectorXd &measurements, LogLevel logLevel)
	{
		double errorSq = 0, error = 0, maxError = 0;
		int ptCnt = 0;
		for (int i = 0; i < mutableCount; i += 2)
		{
			int dc = dataTensor.indices(i,0), f = dataTensor.indices(i,1), s = dataTensor.indices(i,2);
			if (disableFrame(f)) continue;

			Eigen::Vector2d measuredPtN(measurements(i), measurements(i+1));
			Eigen::Vector3d reconPt(tucker(Eigen::Vector3i(dc+0, f, s)), tucker(Eigen::Vector3i(dc+1, f, s)), tucker(Eigen::Vector3i(dc+2, f, s)));
			Eigen::Vector2d ptDiff = measuredPtN-reconPt.hnormalized();

			errorSq += ptDiff.squaredNorm();
			double ptError = ptDiff.norm();
			error += ptError;
			maxError = std::max(maxError, ptError);
			ptCnt++;
		}
		for (int i = mutableCount; i < mutableCount+immutableCount; i += 3)
		{
			int dc = dataTensor.indices(i,0), f = dataTensor.indices(i,1), s = dataTensor.indices(i,2), c = dc/3;
			if (disableFrame(f)) continue;

			Eigen::Vector3d measuredPt(dataTensor.values(i), dataTensor.values(i+1), dataTensor.values(i+2));
			Eigen::Vector3d reconPt(tucker(Eigen::Vector3i(dc+0, f, s)), tucker(Eigen::Vector3i(dc+1, f, s)), tucker(Eigen::Vector3i(dc+2, f, s)));
			Eigen::Vector2d ptDiff = measuredPt.hnormalized()-reconPt.hnormalized();

			errorSq += ptDiff.squaredNorm();
			double ptError = ptDiff.norm();
			error += ptError;
			maxError = std::max(maxError, ptError);
			ptCnt++;
		}
		double Avg = error / ptCnt;
		double RMSE = std::sqrt(errorSq / ptCnt);
		double fitMat = 1 - (std::sqrt(errorSq) / dataTensor.norm());
		if (label)
			LOGC(logLevel, "%s avg error is %fpx, max is %fpx, RMSE %fpx, fit %f\n", label, Avg*PixelFactor, maxError*PixelFactor, RMSE*PixelFactor, fitMat);
		return std::make_tuple(Avg, RMSE, fitMat);
	};
	auto calculateElementErrors = [mutableCount, immutableCount, &disableFrame](const char *label, const Decomp &tucker, const SparseTensor<double, 3> &dataTensor, const Eigen::VectorXd &measurements, Eigen::VectorXd &markerError, Eigen::VectorXi &markerNum, Eigen::VectorXd &frameError, Eigen::VectorXi &frameNum, LogLevel logLevel)
	{
		markerError.resize(tucker.factor<2>().rows());
		markerError.setZero();
		markerNum.resize(tucker.factor<2>().rows());
		markerNum.setZero();
		frameError.resize(tucker.factor<1>().rows());
		frameError.setZero();
		frameNum.resize(tucker.factor<1>().rows());
		frameNum.setZero();
		Eigen::VectorXd cameraErr(tucker.factor<0>().rows()/3);
		Eigen::VectorXi cameraNum(tucker.factor<0>().rows()/3);
		Eigen::VectorXd cameraErrTri(tucker.factor<0>().rows()/3);
		Eigen::VectorXi cameraNumTri(tucker.factor<0>().rows()/3);
		cameraErr.setZero();
		cameraNum.setZero();
		cameraErrTri.setZero();
		cameraNumTri.setZero();
		for (int i = 0; i < mutableCount; i += 2)
		{
			int dc = dataTensor.indices(i,0), f = dataTensor.indices(i,1), s = dataTensor.indices(i,2), c = dc/3;
			if (disableFrame(f)) continue;

			Eigen::Vector2d measuredPtN(measurements(i), measurements(i+1));
			Eigen::Vector3d reconPt(tucker(Eigen::Vector3i(dc+0, f, s)), tucker(Eigen::Vector3i(dc+1, f, s)), tucker(Eigen::Vector3i(dc+2, f, s)));
			double errSq = (measuredPtN-reconPt.hnormalized()).squaredNorm();
			markerError(s) += errSq;
			markerNum(s)++;
			frameError(f) += errSq;
			frameNum(f)++;
			cameraErr(c) += errSq;
			cameraNum(c)++;
		}
		for (int i = mutableCount; i < mutableCount+immutableCount; i += 3)
		{
			int dc = dataTensor.indices(i,0), f = dataTensor.indices(i,1), s = dataTensor.indices(i,2), c = dc/3;
			if (disableFrame(f)) continue;
			Eigen::Vector3d measuredPt(dataTensor.values(i), dataTensor.values(i+1), dataTensor.values(i+2));
			Eigen::Vector3d reconPt(tucker(Eigen::Vector3i(dc+0, f, s)), tucker(Eigen::Vector3i(dc+1, f, s)), tucker(Eigen::Vector3i(dc+2, f, s)));
			double errSq = (measuredPt.hnormalized()-reconPt.hnormalized()).squaredNorm();
			markerError(s) += errSq;
			markerNum(s)++;
			frameError(f) += errSq;
			frameNum(f)++;
			cameraErrTri(c) += errSq;
			cameraNumTri(c)++;
		}
		double minFrameError = 100000000, maxFrameError = 0, avgFrameError = 0;
		int frameCnt = 0;
		for (int f = 0; f < frameNum.size(); f++)
		{
			if (frameNum(f) == 0)
			{ // assert(disableFrame(f) == 1)
				continue;
			}
			frameError(f) = std::sqrt(frameError(f)/frameNum(f));
			minFrameError = std::min(minFrameError, frameError(f));
			maxFrameError = std::max(maxFrameError, frameError(f));
			avgFrameError += frameError(f);
			frameCnt++;
		}
		avgFrameError = avgFrameError/frameCnt;
		markerError.array() = markerError.array().cwiseQuotient(markerNum.cast<double>().array()).cwiseSqrt();

		if (!label)
			return;
		LOGC(logLevel, "%s marker RMSE is avg %f (max %f), frame RMSE is avg %f (min %f, max %f), %d frames with data\n",
			label, markerError.mean()*PixelFactor, markerError.maxCoeff()*PixelFactor,
			avgFrameError*PixelFactor, minFrameError*PixelFactor, maxFrameError*PixelFactor, frameCnt);
		for (int c = 0; c < tucker.factor<0>().rows()/3; c++)
		{
			LOGC(logLevel, "        Camera %d has %f RMSE normally, %f RMSE from tris\n", c,
				std::sqrt(cameraErr(c)/cameraNum(c))*PixelFactor, std::sqrt(cameraErrTri(c)/cameraNumTri(c))*PixelFactor);
		}
	};

	auto enforceMotionIntegrity = [&disableFrame](Eigen::MatrixXd &motion)
	{
		for (int f = 0; f < motion.rows(); f++)
		{
			if (disableFrame(f)) { motion.row(f).setZero(); continue; }
			Eigen::Vector<double,13> transform = motion.row(f);
			auto linear = Eigen::Map<Eigen::Matrix3d>(transform.data());
			//auto translation = Eigen::Map<Eigen::Vector3d>(transform.data()+9);
			// Enforce normed
			transform(12) = 1;
			// Enforce isometry
			Eigen::AngleAxisd fixRot(linear);
			linear = fixRot.toRotationMatrix();
			// Reassign
			motion.row(f) = transform;
		}
	};

	auto setFrameDisabled = [mutableCount, immutableCount, &disableFrame, &recFrameCount, &cameraCalibs](int frame, SparseTensor<double, 3> &dataTensor, Eigen::MatrixXd &motion)
	{
		motion.row(frame).setZero();
		if (disableFrame(frame)) return;
		recFrameCount--;
		disableFrame(frame) = 1;
		for (int i = 0; i < mutableCount; i += 2)
		{
			int dc = dataTensor.indices(i,0), f = dataTensor.indices(i,1), s = dataTensor.indices(i,2), c = dc/3;
			if (frame == f)
			{
				dataTensor.values(i+0) = 0;
				dataTensor.values(i+1) = 0;
				//dataTensor.values(i+0) = cameraCalibs[c].principalPoint.x();
				//dataTensor.values(i+1) = cameraCalibs[c].principalPoint.y();
			}
		}
		for (int i = mutableCount; i < mutableCount+immutableCount; i += 3)
		{
			int dc = dataTensor.indices(i,0), f = dataTensor.indices(i,1), s = dataTensor.indices(i,2), c = dc/3;
			if (frame == f)
			{
				dataTensor.values(i+0) = 0;
				dataTensor.values(i+1) = 0;
				dataTensor.values(i+2) = 0;
				//dataTensor.values(i+0) = cameraCalibs[c].principalPoint.x();
				//dataTensor.values(i+1) = cameraCalibs[c].principalPoint.y();
				//dataTensor.values(i+2) = 1;
			}
		}
	};

	// These are the multiplication indices over which the factors are multiplied (entries of the core matrix)
	std::array<std::array<Eigen::Index, 3>, 13> factorDegrees;
	for (int i = 0; i < 3; i++)
	{
		factorDegrees[i*4+0] = { i, i+0, 0 };
		factorDegrees[i*4+1] = { i, i+3, 1 };
		factorDegrees[i*4+2] = { i, i+6, 2 };
		factorDegrees[i*4+3] = { i, i+9, 3 };
	}
	factorDegrees[12] = { 3, 12, 3 };

	Decomp tucker({ 4, 13, 4 }, factorDegrees);

	// Orders in order: Camera, Motion, Structure

	if (useGTdata && targetGT)
	{ // Use GT data in simulation mode. For debugging the algorithm
		dataTensor = dataTensorGT;
		measurements = measurementsGT;
		//projectiveDepths = projectiveDepthsEstGT;
		projectiveDepths = projectiveDepthsGT;
	}

	// Setup helper structures for tensor
	tucker.prepareTensor(dataTensor);

	// Set known camera factor
	for (int c = 0; c < camCount; c++)
	{
		Eigen::Matrix4d cam = cameraCalibs[c].camera.matrix();
		tucker.factor<0>().middleRows<2>(c*3) = cam.topRows<2>();
		tucker.factor<0>().middleRows<1>(c*3+2) = cam.bottomRows<1>();
	}

	if (useGTsolution && targetGT)
	{
		// Initialise homogeneous flattened motion factor
		for (int f = 0; f < frameCount; f++)
		{
			Eigen::Isometry3d pose = targetGT->frames[f].pose.cast<double>();
			Eigen::Vector<double,13> transform;
			Eigen::Map<Eigen::Matrix3d>(transform.data()) = pose.rotation();
			Eigen::Map<Eigen::Vector3d>(transform.data()+9) = pose.translation();
			transform(12) = 1;
			tucker.factor<1>().row(f) = transform;
		}

		// Initialise homogeneous structure factor
		for (int m = 0; m < markerCount; m++)
		{
			tucker.factor<2>().row(m) = targetGT->markers[m].cast<double>().homogeneous();
		}
	}
	else
	{
		// Initialise with random homogeneous motion factor
		tucker.factor<1>().setRandom();
		tucker.factor<1>().rightCols<1>().setConstant(1);

		// Initialise with homogeneous near-zero structure factor
		tucker.factor<2>().setRandom();
		tucker.factor<2>() *= 0.05;
		tucker.factor<2>().rightCols<1>().setConstant(1);
	}

	double lastRMSE = 0;
	Eigen::VectorXd projDepths = Eigen::VectorXd::Ones(mutableCount/2);
	if (mutableCount == 0)
		projDepths.resize(1);

	{
		Eigen::VectorXd markerErr, frameErr;
		Eigen::VectorXi markerNum, frameNum;
		calculateElementErrors("After initialisation", tucker, dataTensor, measurements, markerErr, markerNum, frameErr, frameNum, LTrace);
		calculateErrors("After initialisation", tucker, dataTensor, measurements, LTrace);

		if (targetGT)
		{
			LOGC(LTrace, "    GT Projective Depths mean %f, min %f, max %f\n",
				projectiveDepthsGT.mean(), projectiveDepthsGT.minCoeff(), projectiveDepthsGT.maxCoeff());

			LOGC(LTrace, "    GT Estimated Projective Depths mean %f, min %f, max %f\n",
				projectiveDepthsEstGT.mean(), projectiveDepthsEstGT.minCoeff(), projectiveDepthsEstGT.maxCoeff());

			Eigen::VectorXd GTestProjDepthsErr = projectiveDepthsEstGT-projectiveDepthsGT;
			Eigen::VectorXd GTestProjDepthsErrAbs = GTestProjDepthsErr.array().abs();
			LOGC(LTrace, "        GT Estimation Error mean %f, min %f, max %f, dir %f\n",
				GTestProjDepthsErrAbs.mean(), GTestProjDepthsErrAbs.minCoeff(), GTestProjDepthsErrAbs.maxCoeff(), GTestProjDepthsErr.mean());
		}

		LOGC(LDebug, "    Estimated Projective Depths mean %f, min %f, max %f\n",
			projectiveDepths.mean(), projectiveDepths.minCoeff(), projectiveDepths.maxCoeff());

		if (targetGT)
		{
			Eigen::VectorXd estProjDepthsErr = projectiveDepths-projectiveDepthsGT;
			Eigen::VectorXd estProjDepthsErrAbs = estProjDepthsErr.array().abs();
			LOGC(LTrace, "        Estimation Error mean %f, min %f, max %f, dir %f\n",
				estProjDepthsErrAbs.mean(), estProjDepthsErrAbs.minCoeff(), estProjDepthsErrAbs.maxCoeff(), estProjDepthsErr.mean());
		}

		getProjectiveDepths(tucker, dataTensor, projDepths);
		LOGC(LDebug, "    Initial Reconstructed Projective Depths mean %f, min %f, max %f\n",
			projDepths.mean(), projDepths.minCoeff(), projDepths.maxCoeff());

		if (targetGT)
		{
			Eigen::VectorXd InitProjDepthsErr = projDepths-projectiveDepthsGT;
			Eigen::VectorXd InitProjDepthsErrAbs = InitProjDepthsErr.array().abs();
			LOGC(LTrace, "        Initial Error mean %f, min %f, max %f, dir %f\n",
				InitProjDepthsErrAbs.mean(), InitProjDepthsErrAbs.minCoeff(), InitProjDepthsErrAbs.maxCoeff(), InitProjDepthsErr.mean());
		}

		enforceMotionIntegrity(tucker.factor<1>());

		calculateErrors("After motion constraints", tucker, dataTensor, measurements, LTrace);

		getProjectiveDepths(tucker, dataTensor, projDepths);
		LOGC(LTrace, "    Constrained Projective Depths mean %f, min %f, max %f\n",
			projDepths.mean(), projDepths.minCoeff(), projDepths.maxCoeff());

		if (targetGT)
		{
			Eigen::VectorXd ConstrProjDepthsErr = projDepths-projectiveDepthsGT;
			Eigen::VectorXd ConstrProjDepthsErrAbs = ConstrProjDepthsErr.array().abs();
			LOGC(LTrace, "        Constrained Error mean %f, min %f, max %f, dir %f\n",
				ConstrProjDepthsErrAbs.mean(), ConstrProjDepthsErrAbs.minCoeff(), ConstrProjDepthsErrAbs.maxCoeff(), ConstrProjDepthsErr.mean());
		}
	}

	{
		Eigen::VectorXd markerErr, frameErr;
		Eigen::VectorXi markerNum, frameNum;
		calculateElementErrors("    ! Start", tucker, dataTensor, measurements, markerErr, markerNum, frameErr, frameNum, LTrace);

		for (int m = 0; m < markerCount; m++)
		{
			if (markerNum(m) < 15)
			{
				// TODO: Implement?
				LOGC(LDebug, "    -- Marker %d/%d had error %f over only %d points, considering to remove\n", m, markerCount, markerErr(m), markerNum(m));
			}
		}
		for (int f = 0; f < frameCount; f++)
		{
			if (frameNum(f) < 8)
			{
				setFrameDisabled(f, dataTensor, tucker.factor<1>());
				LOGC(LDebug, "    -- Frame %d/%d had error %f over only %d points, removing\n", f, frameCount, frameErr(f), frameNum(f));
			}
		}
	}

	for (int i = 0; i < pMaxIteration; i++)
	{
		LOGC(LDebug, "Modified Tucker Iteration %d:\n", i);

		{
			Eigen::VectorXd pointNorm = tucker.factor<2>().rightCols<1>();
			Eigen::VectorXd row1Norm = tucker.factor<1>().middleCols<3>(0).rowwise().norm();
			Eigen::VectorXd row2Norm = tucker.factor<1>().middleCols<3>(3).rowwise().norm();
			Eigen::VectorXd row3Norm = tucker.factor<1>().middleCols<3>(6).rowwise().norm();
			LOGC(LDebug, "        Motion Row Norms are %f, %f, %f, Point Norm is %f\n", row1Norm.mean(), row2Norm.mean(), row3Norm.mean(), pointNorm.mean());
		}

		// Structure update
		tucker.updateFactor(2, dataTensor);
		calculateErrors("    After structure update", tucker, dataTensor, measurements, LDebug);

		{ // Enforce homogeneous coordinates in structure factor
			auto scales = tucker.factor<2>().rightCols<1>();
			scales = (scales.array() == 0).select(1, scales);
			for (int m = 0; m < markerCount; m++)
				tucker.factor<2>().row(m) = tucker.factor<2>().row(m).hnormalized().homogeneous();
			// Transform translations according to point scales
			tucker.factor<1>().middleCols<3>(9).array() *= scales.mean();
		}
		calculateErrors("    After structure constraint", tucker, dataTensor, measurements, LDebug);

		// Motion update
		tucker.updateFactor(1, dataTensor);
		calculateErrors("    After motion update", tucker, dataTensor, measurements, LDebug);

		// Enforce motion isometry and norm constraints
		enforceMotionIntegrity(tucker.factor<1>());
		calculateErrors("    After motion constraints", tucker, dataTensor, measurements, LDebug);

		LOGC(LDebug, "    Current stats:\n");
		{
			/* Eigen::VectorXd structureN = tucker.factor<2>().rightCols<1>();
			LOGC(LDebug, "        Structure homogeneous coordinates mean %f, min %f, max %f\n", structureN.mean(), structureN.minCoeff(), structureN.maxCoeff());
			Eigen::VectorXd motionN = tucker.factor<1>().rightCols<1>();
			LOGC(LDebug, "        Motion homogeneous coordinates mean %f, min %f, max %f\n", motionN.mean(), motionN.minCoeff(), motionN.maxCoeff()); */
			getProjectiveDepths(tucker, dataTensor, projDepths);
			LOGC(LDebug, "        Projective Depths mean %f, min %f, max %f\n",
				projDepths.mean(), projDepths.minCoeff(), projDepths.maxCoeff());
			if (targetGT)
				LOGC(LDebug, "        GT Projective Depths mean %f, min %f, max %f\n",
					projectiveDepthsGT.mean(), projectiveDepthsGT.minCoeff(), projectiveDepthsGT.maxCoeff());
		}

		{
			Eigen::VectorXd markerErr, frameErr;
			Eigen::VectorXi markerNum, frameNum;
			calculateElementErrors("    ! Inter", tucker, dataTensor, measurements, markerErr, markerNum, frameErr, frameNum, LDebug);
			recordSolution(tucker, target, frameErr, frameNum);

			// Attempt to not only find the best solution, but assemble it from others
			// This is because even the best solution will have random outliers that really don't need to be outliers
			// E.g. other solutions will have a locally consistent solution for them
			// So assembling them IS possible with some further work, but might not be worth it

			//debugSolutionByFrames(tucker, target, frameErr, frameNum);
			//std::this_thread::sleep_for(std::chrono::milliseconds(500));

/* 			std::this_thread::sleep_for(std::chrono::seconds(2));

			thread_local std::vector<std::pair<Eigen::Isometry3d, double>> bestPoses;
			bestPoses.resize(frameCount);
			for (int f = 0; f < frameCount; f++)
			{
				auto &best = bestPoses[f];

				Eigen::Vector<double,13> transform = tucker.factor<1>().row(f);
				Eigen::Isometry3d pose;
				pose.linear() = Eigen::Map<Eigen::Matrix3d>(transform.data());
				pose.translation() = Eigen::Map<Eigen::Vector3d>(transform.data()+9);
 */
				/* if (std::get<1>(best) == 0 || std::get<1>(best) > frameErr(f))
				{
					std::get<0>(best) = pose;
					std::get<1>(best) = frameErr(f);
					continue;
				}
				else
				{
					pose = std::get<0>(best);
					Eigen::Map<Eigen::Matrix3d>(transform.data()) = pose.linear();
					Eigen::Map<Eigen::Vector3d>(transform.data()+9) = pose.translation();
				} */
				/* if (std::get<1>(best) == 0)
				{
					std::get<0>(best) = pose;
					std::get<1>(best) = frameErr(f);
					continue;
				}

				double errB = 0, errC = 0;
				for (auto &sample : target.frames[f].samples)
				{
					Eigen::Vector3d transPointB = std::get<0>(best) * tucker.factor<2>().row(target.markerMap[sample.marker]).transpose().homogeneous();
					Eigen::Vector4d projPointB = cameraCalibs[sample.camera].camera * transPointB.homogeneous();
					errB += (projPointB.hnormalized().head<2>() - sample.point.cast<double>()).norm();
					Eigen::Vector3d transPointC = pose * tucker.factor<2>().row(target.markerMap[sample.marker]).transpose().homogeneous();
					Eigen::Vector4d projPointC = cameraCalibs[sample.camera].camera * transPointC.homogeneous();
					errC += (projPointC.hnormalized().head<2>() - sample.point.cast<double>()).norm();
				}
				if (errB > errC)
				{
					pose = std::get<0>(best);
					Eigen::Map<Eigen::Matrix3d>(transform.data()) = pose.linear();
					Eigen::Map<Eigen::Vector3d>(transform.data()+9) = pose.translation();
					if (errB > errC*2)
						LOGC(LDebug, "Frame Index %d: Replaced best /w error %f with error %f\n", f, errB, errC);
				}
			}
			LOGC(LDebug, "Finished replacing outliers with best!\n");

			calculateElementErrors("    ! Inter", tucker, dataTensor, measurements, markerErr, markerNum, frameErr, frameNum);
			debugSolutionByFrames(tucker, target, frameErr, frameNum);
			std::this_thread::sleep_for(std::chrono::seconds(2)); */
		}

		if (i > pCorrectIteration && !useGTdata)
		{
			correctProjectiveDepth(tucker, dataTensor, measurements, projectiveDepths);

			calculateErrors("    ! After correcting projective depths", tucker, dataTensor, measurements, LDebug);
		}

		if (i > pDropIteration)
		{
			Eigen::VectorXd markerErr, frameErr;
			Eigen::VectorXi markerNum, frameNum;
			calculateElementErrors("    ! DropQ", tucker, dataTensor, measurements, markerErr, markerNum, frameErr, frameNum, LDebug);

			int maxMarkerIndex;
			double maxMarkerErr = markerErr.maxCoeff(&maxMarkerIndex), meanMarkerErr = markerErr.mean();
			if (maxMarkerErr > meanMarkerErr*5)
			{
				LOGC(LDebug, "        Could remove marker %d with reprojection error of max %.2f, avg %.2f\n", maxMarkerIndex, maxMarkerErr, meanMarkerErr);
			}

			int maxFrameIndex;
			double maxFrameErr = frameErr.maxCoeff(&maxFrameIndex), meanFrameErr = frameErr.mean();
			int alreadyRemoved = 0, newlyRemoved = 0, remaining = 0;
			for (int f = 0; f < frameCount; f++)
			{
				if (disableFrame(f))
				{
					alreadyRemoved++;
					LOGC(LTrace, "        Disabled frame %d had error %f over %d points!\n", f, frameErr(f), frameNum(f));
					continue;
				}
				if (frameErr(f) > std::max(maxFrameErr/2, meanFrameErr*5))
				{
					setFrameDisabled(f, dataTensor, tucker.factor<1>());
					LOGC(LDebug, "    -- Frame %d had error %f over %d points, removing\n", f, frameErr(f), frameNum(f));
					newlyRemoved++;
				}
				else
					remaining++;
			}
			LOGC(LDebug, "   Have %d frames remaining, %d newly removed, %d previously removed\n", remaining, newlyRemoved, alreadyRemoved);
		}


		double prevRMSE = lastRMSE;
		auto results = calculateErrors("    Reconstructed", tucker, dataTensor, measurements, LDebug);
		lastRMSE = std::get<1>(results);
		if (i > pMinAbortIteration && std::abs(prevRMSE-lastRMSE) <= 0.0001)
			break;
	}

	{ // Analyze iterations for best solution

		SolutionCandidate &best = solutionCandidates.front();
		SolutionCandidate &bestGT = solutionCandidates.front();
		for (auto &cand : solutionCandidates)
		{
			if (cand.error < best.error)
				best = cand;
			if (cand.markerErrorGT < bestGT.markerErrorGT)
				bestGT = cand;
		}

		tucker.factor<1>() = best.motion;
		tucker.factor<2>() = best.structure;
		disableFrame = best.disableFrame;
		disableMarker = best.disableMarker;
		LOGC(LDebug, "Best sample rating: %.4fpx error, %.4fmm GT Marker RMSE\n",
			best.error, best.markerErrorGT);
		LOGC(LDebug, "Best GT sample rating: %.4fpx error, %.4fmm GT Marker RMSE\n",
			bestGT.error, bestGT.markerErrorGT);
	}

	if (targetGT)
	{
		Eigen::VectorXd markerErr, frameErr;
		Eigen::VectorXi markerNum, frameNum;
		calculateElementErrors("Error on GT", tucker, dataTensor, measurementsGT, markerErr, markerNum, frameErr, frameNum, LDebug);
	}

	Eigen::VectorXd markerErr, frameErr;
	Eigen::VectorXi markerNum, frameNum;
	calculateElementErrors("Error on Data", tucker, dataTensor, measurements, markerErr, markerNum, frameErr, frameNum, LDebug);

	LOGC(LDebug, "RMSE of reconstructed target is %fpx\n", frameErr.mean()*PixelFactor);

	//Eigen::MatrixXd &cameras = tucker.factor<0>();
	Eigen::MatrixXd &motion = tucker.factor<1>();
	Eigen::MatrixXd &structure = tucker.factor<2>();

	{

		int maxMarkerIndex;
		double maxMarkerErr = markerErr.maxCoeff(&maxMarkerIndex), meanMarkerErr = markerErr.mean();
		if (maxMarkerErr > meanMarkerErr*5)
		{
			LOGC(LDebug, "    Could remove marker %d with reprojection error of max %.2f, avg %.2f\n", maxMarkerIndex, maxMarkerErr, meanMarkerErr);
		}
		double meanFrameErr = frameErr.mean();
		for (int f = 0; f < frameCount; f++)
		{
			if (frameErr(f) > meanFrameErr*5)
				LOGC(LDebug, "    Frame %d had error %f over all %d points, removed? %d\n", f, frameErr(f), frameNum(f), disableFrame(f));
		}

		for (int m = 0; m < markerCount; m++)
			LOGC(LTrace, "    Marker %d had error %f over all %d points\n", m, markerErr(m), markerNum(m));
		for (int f = 0; f < frameCount; f++)
			LOGC(LTrace, "    Frame %d had error %f over all %d points\n", f, frameErr(f), frameNum(f));
	}

	{ // Update frames and markers with solution, remove disabled frame and markers
		target.totalSamples = 0;
		auto frame = target.frames.begin();
		for (int f = 0; f < frameCount; f++)
		{
			if (disableFrame(f))
			{
				frame = target.frames.erase(frame);
				continue;
			}
			// Update frame with solution
			Eigen::Vector<double,13> transform = motion.row(f);
			frame->pose.linear() = Eigen::Map<Eigen::Matrix3d>(transform.data()).cast<float>();
			frame->pose.translation() = Eigen::Map<Eigen::Vector3d>(transform.data()+9).cast<float>();
			frame->error = (float)frameErr(f);
			// Delete observations of markers which were disabled
			for (auto sample = frame->samples.begin(); sample != frame->samples.end();)
			{
				if (disableMarker(target.markerMap[sample->marker]))
					sample = frame->samples.erase(sample);
				else sample++;
			}

			target.totalSamples += frame->samples.size();
			frame++;
		}

		int index = 0; // New index of current marker m
		for (int m = 0; m < markerCount; m++)
		{
			if (disableMarker(m))
			{
				LOGC(LDebug, "Set marker %d to be disabled!\n", m);
				target.markers.erase(target.markers.begin()+index);
				for (auto mod = target.markerMap.begin(); mod != target.markerMap.end();)
				{ // Update markerMap to account for deleted marker
					if (mod->second == index)
						mod = target.markerMap.erase(mod);
					else
					{
						if (mod->second > index)
							mod->second--;
						mod++;
					}
				}
				continue;
			}
			Eigen::Vector4d pt = structure.row(m); // gtTransform *
			target.markers[index] = pt.hnormalized().cast<float>();
			index++;
		}
	}

	return true;
}

/**
 * Accumulate all observations of the target in a data tensor [cameras, frames, markers]
 * Triangulates all triangulatable observations and puts them with their projective depth in an immutable section
 * Approximates projective depth of all other observations using those triangulations and puts them in the mutable section
 */
static std::tuple<int,int,int,int> buildDataTensor(const ObsTarget &target, const std::vector<CameraCalib> &cameraCalibs, SparseTensor<double, 3> &dataTensor, Eigen::VectorXd &measurements, Eigen::VectorXd &projectiveDepths)
{
	int markerCount = target.markers.size();
	int camCount = cameraCalibs.size();
	int frameCount = target.frames.size();
	int sampleCount = target.totalSamples;

	int triPointCount = 0, triSourceCount = 0;
	std::vector<std::map<int, Eigen::Vector3f>> frameTris(frameCount);
	// TODO: Switch from map to vector<pair> - but not critical at all
	Eigen::MatrixXd approxProjectiveDepths(frameCount, camCount);
	{
		// For counting observations for each marker each frame
		std::vector<int> tempMarkerObs;

		// Setup interfacing structures for triangulation
		std::vector<std::vector<Eigen::Vector2f>> blobContainer(camCount);
		std::vector<std::vector<Eigen::Vector2f> const *> points2D(camCount);
		for (int c = 0; c < camCount; c++)
		{
			blobContainer[c].resize(1);		 // Used to store a single blob for each camera involved with a point
			points2D[c] = &blobContainer[c]; // Just interfacing
		}
		TriangulatedPoint triPoint(Eigen::Vector3f::Zero(), 0, 10, camCount);

		int f = 0;
		for (auto frame = target.frames.begin(); frame != target.frames.end(); frame++, f++)
		{
			// Count marker observations this frame
			tempMarkerObs.clear();
			tempMarkerObs.resize(markerCount);
			for (const auto &sample : frame->samples)
				tempMarkerObs[target.markerMap.at(sample.marker)]++;

			// Find triangulatable markers and triangulate them
			for (int m = 0; m < markerCount; m++)
			{
				if (tempMarkerObs[m] <= 1) continue;
				// Found triangulatable point
				triPointCount++;
				triSourceCount += tempMarkerObs[m];

				// Reset point
				for (int c = 0; c < camCount; c++)
					triPoint.blobs[c] = InvalidBlob;
				// Enter 2D observations
				for (auto &sample : frame->samples)
				{
					if (target.markerMap.at(sample.marker) != m)
						continue;
					blobContainer[sample.camera][0] = undistortPoint(cameraCalibs[sample.camera], sample.point);
					triPoint.blobs[sample.camera] = 0;
				}
				// Calculate optimal triangulation
				//Eigen::Vector3f tri = refineTriangulation<float, float, double>(points2D, cameraCalibs, triPoint);
				Eigen::Vector3f tri = refineTriangulationIterative<float, float, double>(points2D, cameraCalibs, triPoint, 5);
				frameTris[f][m] = tri;
			}

			// Get 2D center of marker group for each camera to triangulate
			std::vector<Eigen::Vector2f> centerPos2D(camCount, Eigen::Vector2f::Zero());
			std::vector<int> obsCount2D(camCount);
			for (auto &sample : frame->samples)
			{
				centerPos2D[sample.camera] += sample.point;
				obsCount2D[sample.camera]++;
			}

			// Reset point
			for (int c = 0; c < camCount; c++)
				triPoint.blobs[c] = InvalidBlob;
			int viewCnt = 0;
			for (int c = 0; c < camCount; c++)
			{
				if (obsCount2D[c] == 0) continue;
				blobContainer[c][0] = centerPos2D[c] / obsCount2D[c];
				triPoint.blobs[c] = 0;
				viewCnt++;
			}
			if (viewCnt > 1)
			{
				Eigen::Vector3d centerTri = refineTriangulationIterative<float, float, double>(points2D, cameraCalibs, triPoint, 5).cast<double>();
				for (int c = 0; c < camCount; c++)
					approxProjectiveDepths(f, c) = (cameraCalibs[c].camera * centerTri.homogeneous()).w();
			}
			else
			{
				// TODO: No references for this frame, fill later by averaging between past and future frames?
				LOGC(LDarn, "Unable to determine approximate projective depth for frame %d!\n", f);
				for (int c = 0; c < camCount; c++)
					approxProjectiveDepths(f, c) = 1;
			}
		}
	}

	// Factorisation data is split in two sections, first of observations that couldn't be used for triangulation
	//  These have unknown projective depth, which will need to be iteratively adapted (mutable)
	// Second half is observations with triangulations (so data for all cameras is inferred)
	//  Since projective depth is known they do not need to be corrected and are immutable
	int mutableCount = (sampleCount - triSourceCount) * 2; // 2D coordinates, sample points per camera
	int immutableCount = triPointCount * camCount * 3; // 3D coordinates, tri points for all cameras
	int dataCount = mutableCount + immutableCount;

	// Estimate projective depth from base estimate, plus offsets from nearby actual triangulated markers
	// Here because it's used for both normal and ground truth data
	std::vector<int> frame3DIndex(frameCount);
	auto estimateProjectiveDepth = [&](int f, int c, int m, const SparseTensor<double, 3> &data)
	{
		int frameIndex = frame3DIndex[f] + 3*c;
		int start = mutableCount;
		int end = mutableCount + immutableCount;

		int t = frameIndex;
		while (t < end && data.indices(t, 1) == f && data.indices(t, 2) != m)
			t += 3*camCount;
		bool hasCur = t < end && data.indices(t, 1) == f && data.indices(t, 2) == m;

		if (hasCur)
		{ // Know projective depth from triangulation, no estimation needed
			return std::pair<double, int>(data.values(t+2), t);
		}

		int n = t;
		while (n < end && data.indices(n, 1)-f < 50 && data.indices(n, 2) != m)
			n += 3*camCount;
		bool hasNext = n < end && data.indices(n, 2) == m;

		int p = frameIndex;
		do { p -= 3*camCount; }
		while (p >= start && f-data.indices(p, 1) < 50 && data.indices(p, 2) != m);
		bool hasPrev = p >= start && data.indices(p, 2) == m;

		double projectiveDepth = approxProjectiveDepths(f, c);
		if (hasPrev && hasNext)
		{  // Interpolate between previous and next frames
			float fac1 = 1.0f/(f - data.indices(p, 1));
			float fac2 = 1.0f/(data.indices(n, 1) - f);
			double off1 = data.values(p+2) + approxProjectiveDepths(data.indices(p, 1), c);
			double off2 = data.values(n+2) + approxProjectiveDepths(data.indices(n, 1), c);
			projectiveDepth += (off2*fac2 + off1*fac1) / (fac1+fac2);
		}
		else if (hasPrev)
		{// Estimate from previous frame
			projectiveDepth += data.values(p+2) - approxProjectiveDepths(data.indices(p, 1), c);
		}
		else if (hasNext)
		{ // Estimate from future frames (+past?)
			projectiveDepth += data.values(n+2) - approxProjectiveDepths(data.indices(n, 1), c);
		}

		return std::pair<double, int>(projectiveDepth, -1);
	};

	{ // Initialise dataTensor, measurements, projectiveDepths

		measurements.resize(mutableCount);
		projectiveDepths.resize(mutableCount/2);

		// Initialise sparse data structures
		dataTensor.dimensions = { camCount*3, frameCount, markerCount };
		dataTensor.indices.resize(dataCount, 3);
		dataTensor.values.resize(dataCount);

		// Indices into sparse data structures in dataTensor for 3D section
		int indexB = mutableCount;
		for (int f = 0; f < target.frames.size(); f++)
		{
			frame3DIndex[f] = indexB;
			// Enter triangulated point data first
			for (auto &tri : frameTris[f])
			{
				int m = tri.first;
				Eigen::Vector3d triPt = tri.second.cast<double>();
				for (int c = 0; c < camCount; c++)
				{ // Enter reprojected coordinates, NOT hnormalised (linear tensor product)
					Eigen::Vector4d projected = cameraCalibs[c].camera * triPt.homogeneous();
					dataTensor.indices.row(indexB + c*3+0) = Eigen::Vector3i(c*3+0, f, m);
					dataTensor.indices.row(indexB + c*3+1) = Eigen::Vector3i(c*3+1, f, m);
					dataTensor.indices.row(indexB + c*3+2) = Eigen::Vector3i(c*3+2, f, m);
					dataTensor.values(indexB + c*3+0) = projected.x();
					dataTensor.values(indexB + c*3+1) = projected.y();
					dataTensor.values(indexB + c*3+2) = projected.w();
				}
				indexB += 3*camCount;
			}
		}

		int indexA = 0;
		int f = -1;
		for (auto &frame : target.frames)
		{
			f++;

			// Now enter 2D observations, also overwrite 3D reprojection with observations if available
			for (auto &sample : frame.samples)
			{
				int m = target.markerMap.at(sample.marker);
				Eigen::Vector2d point = undistortPoint(cameraCalibs[sample.camera], sample.point).cast<double>();

				auto proj = estimateProjectiveDepth(f, sample.camera, m, dataTensor);
				if (proj.second >= 0)
				{ // Point has been triangulated
					dataTensor.values(proj.second+0) = point.x()*proj.first;
					dataTensor.values(proj.second+1) = point.y()*proj.first;
					continue;
				}

				// Record into mutable (first) section of data tensor with 2D coords
				dataTensor.indices.row(indexA+0) = Eigen::Vector3i(sample.camera*3+0, f, m);
				dataTensor.values(indexA+0) = point.x()*proj.first;
				dataTensor.indices.row(indexA+1) = Eigen::Vector3i(sample.camera*3+1, f, m);
				dataTensor.values(indexA+1) = point.y()*proj.first;

				// Record data for iterative projectiveDepth estimation
				measurements(indexA+0) = point.x();
				measurements(indexA+1) = point.y();
				projectiveDepths(indexA/2) = proj.first;
				indexA += 2;
			}
		}

		// Update derived indexing structures
		dataTensor.update();
	}

	return std::tuple<int,int,int,int>(triPointCount, triSourceCount, mutableCount, immutableCount);
}