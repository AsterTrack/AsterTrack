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

#include "pipeline.hpp"

#include "util/log.hpp"

#include "util/eigenalg.hpp"

#include "point/sequences2D.hpp"
#include "point/sequence_data.inl"
#include "calib/obs_data.inl"
#include "calib/camera_system.inl"

/**
 * Main processing pipeline for tracking and calibration
 * Currently split in explicit phases, might be organically merged in the future
 */

void InitPipelineStreaming(PipelineState &pipeline)
{
	std::unique_lock pipeline_lock(pipeline.pipelineLock);

	// Init realtime subsystems
	InitTrackingPipeline(pipeline);
}

void ResetPipelineStreaming(PipelineState &pipeline)
{
	std::unique_lock pipeline_lock(pipeline.pipelineLock);

	// Reset realtime subsystems
	ResetTrackingPipeline(pipeline);
}

static void DeletePipelineData(PipelineState &pipeline)
{
	// TODO: Rework when pipeline data is reset to not accidentally loose data
	// Currently, we resets recorded frames, target calibration, samples, etc., basically everything
	// Preferrably those were managed separately, so they can be cleaned up, but independant of whether the cameras are on or not
	// So e.g. target calibration is cleaned up after use by explicit action of user to close it
	// Continuous calibration automatically keeps only X samples
	// Recorded sections are kept until user deletes it
	// frameRecords can be configured to automatically keep only X minutes except if data is still used for any of the above
	// And internal frame indexing (frameNum) needs a rework anyway, so better to keep it running between streaming start/stops

	// Delete data
	pipeline.seqDatabase.contextualLock()->clear();
	pipeline.record.frames.cull_clear(); // Non-blocking, might need another delete_culled later
	pipeline.record.frames.delete_culled(); // If views into frameRecords still exist, this won't delete those blocks
	pipeline.frameNum = -1;
	for (auto &imu : pipeline.record.imus)
	{
		imu->samplesRaw.cull_clear();
		imu->samplesFused.cull_clear();
	}
	for (auto &imu : pipeline.record.imus)
	{
		imu->samplesRaw.delete_culled();
		imu->samplesFused.delete_culled();
	}
}

static void DeletePipelineSetup(PipelineState &pipeline)
{
	pipeline.cameras.clear();
	pipeline.record.imus.clear();
	pipeline.simulation.contextualLock()->resetState();

}

void ResetPipelineState(PipelineState &pipeline)
{
	std::unique_lock pipeline_lock(pipeline.pipelineLock);

	// Reset all subsystems
	ResetTrackingPipeline(pipeline);
	ResetTargetCalibration(pipeline);
	ResetPointCalibration(pipeline);

	// Delete setup and data
	DeletePipelineSetup(pipeline);
	DeletePipelineData(pipeline);
}

void ResetPipelineData(PipelineState &pipeline)
{
	std::unique_lock pipeline_lock(pipeline.pipelineLock);

	// Reset all subsystems
	ResetTrackingPipeline(pipeline);
	ResetTargetCalibration(pipeline);
	ResetPointCalibration(pipeline);

	// Delete data
	DeletePipelineData(pipeline);
}

/**
 * Handles a new frame and returns it once processed
 */
void ProcessFrame(PipelineState &pipeline, std::shared_ptr<FrameRecord> frame)
{
	std::unique_lock pipeline_lock(pipeline.pipelineLock);

	// Accumulate cameras to update
	std::vector<CameraPipeline*> cameras;
	cameras.reserve(pipeline.cameras.size());
	bool fullyCalibrated = true;
	for (auto &cam : pipeline.cameras)
	{
		if (cam->calib.invalid())
			fullyCalibrated = false;
		if (cam->index >= frame->cameras.size())
			continue; // May have been added since the frame was recorded
		auto &record = frame->cameras[cam->index];
		PreprocessCameraData(cam->calib, record);
		cameras.push_back(cam.get());
	}
	// TODO: Some systems might still break if this subset cameras != pipeline.cameras - verify they work
	// But they have been designed to support that by using cameras[x]->index for storage
	// However, temporary data uses the same indexing as this subset of cameras

	if (fullyCalibrated)
	{
		bool targetTracking = pipeline.phase == PHASE_Tracking || pipeline.phase == PHASE_Automatic;
		UpdateTrackingPipeline(pipeline, cameras, frame, targetTracking);
	}

	// TODO: Integrate tracking and calibrations into the same pipeline without phases, just hints
	// Tracking always runs, which is useful for e.g. interactive target calibration in VR
	// Clusters (3D and 2D combined) that can't be identified as a known target after a while would be flagged
	// These can then either automatically or manually be marked for target calibration
	// Point calibration should also be mostly implicit (see below)
	// Sequence2D should then only run when explicitly requested by camera calibration UI or target calibration 

	if (pipeline.recordSequences)
	{ // Recording sequences is desired by point or target calibration

		auto lock = folly::detail::lock(folly::detail::wlock(pipeline.calibration), folly::detail::wlock(pipeline.seqDatabase));
		std::get<0>(lock)->verifyCameraCount(pipeline.cameras.size()); // Keep camera count up-to-date
		std::get<0>(lock)->verifyCameraCount(pipeline.cameras.size()); // Keep camera count up-to-date

		auto &params = pipeline.sequenceParams.get(pipeline.phase == PHASE_Calibration_Point? 0 : 1);

		bool update = false;
		for (auto &cam : cameras)
		{
			auto record = frame->cameras[cam->index];
			if (!record.simulation.points2GTMarker.empty())
			{
				assert(record.simulation.points2GTMarker.size() == record.points2D.size());
			}
			bool seqAdded = updateSequenceCaptures(params, *std::get<0>(lock), *std::get<1>(lock), frame->num,
				cam->index, record.points2D, record.rawPoints2D, record.properties,
				record.simulation.points2GTMarker);
			update = update || seqAdded;
		}

		static int periodicCheck = 0;
		periodicCheck = (periodicCheck+1)%1000;
		checkSequenceHealth(params, *std::get<0>(lock), *std::get<1>(lock), frame->num, update || (periodicCheck == 0));
	}

	if (pipeline.phase == PHASE_Calibration_Point || pipeline.phase == PHASE_Automatic)
	{ // TODO: Reformulate camera calibration for future continuous calibration
		// Then point calibration would just being explicit instructions by UI to camera calibration and sequence2D
		// And continuous calibration would be a background algorithm to optimise camera calibration based on tracking data
		UpdatePointCalibration(pipeline, cameras, frame);
	}

	if (pipeline.phase == PHASE_Calibration_Target)// || pipeline.phase == PHASE_Automatic)
	{
		UpdateTargetCalibration(pipeline, cameras, frame->num);
	}

	frame->finishedProcessing = true;
	pipeline.frameNum = frame->num;
}

void PreprocessCameraData(const CameraCalib &calib, CameraFrameRecord &record)
{
	record.points2D.clear();
	record.points2D.reserve(record.rawPoints2D.size());
	if (calib.valid())
	{ // Undistort points based on calibration
		for (int p = 0; p < record.rawPoints2D.size(); p++)
			record.points2D.push_back(undistortPoint(calib, record.rawPoints2D[p]));
	}
	else
	{ // Initialise with raw points if starting with no calibration
		record.points2D.insert(record.points2D.begin(), record.rawPoints2D.begin(), record.rawPoints2D.end());
	}
}

void PreprocessFrame(const PipelineState &pipeline, FrameRecord &record)
{
	assert(record.cameras.size() == pipeline.cameras.size());
	for (auto &cam : pipeline.cameras)
	{
		PreprocessCameraData(cam->calib, record.cameras[cam->index]);
	}
}

/* For simulation/replay to jump to a specific frame - caller has to make sure no more queued frames will be processed */
void AdoptFrameRecordState(PipelineState &pipeline, const FrameRecord &frameRecord)
{
	std::unique_lock pipeline_lock(pipeline.pipelineLock);

	{ // Initialise the given frame in case it hasn't been replayed yet
		auto framesRecord = pipeline.record.frames.getView();
		if (framesRecord.size() <= frameRecord.num || !framesRecord[frameRecord.num] || framesRecord[frameRecord.num].get() != &frameRecord)
			pipeline.record.frames.insert(frameRecord.num, std::make_shared<FrameRecord>(frameRecord));
	}
	pipeline.frameNum = frameRecord.num; // Next frame will be processed

	// Point and Target calibration pipeline can just stay as is, as long as frame records stays
	// We will be overwriting frame records, so technically leaving them as-is is not thread-safe

	// Mark all trackers as dormant
	std::move(pipeline.tracking.trackedTargets.begin(), pipeline.tracking.trackedTargets.end(), std::back_inserter(pipeline.tracking.dormantTargets));
	pipeline.tracking.trackedTargets.clear();
	std::move(pipeline.tracking.trackedMarkers.begin(), pipeline.tracking.trackedMarkers.end(), std::back_inserter(pipeline.tracking.dormantMarkers));
	pipeline.tracking.trackedMarkers.clear();

	// Adopt state for all recorded trackers in that frame to kickstart tracking
	for (auto &targetRecord : frameRecord.trackers)
	{
		auto dormantIt = std::find_if(pipeline.tracking.dormantTargets.begin(), pipeline.tracking.dormantTargets.end(),
			[&](auto &trk){ return trk.id == targetRecord.id; });
		if (dormantIt == pipeline.tracking.dormantTargets.end()) continue;
		pipeline.tracking.trackedTargets.emplace_back(std::move(*dormantIt), targetRecord.poseObserved, frameRecord.time, frameRecord.num, pipeline.params.track);
		pipeline.tracking.dormantTargets.erase(dormantIt);
	}
	// TODO: Adopt tracked markers
}


/* General functions */

/**
 * Log the parameters and inferred properties of the camera calibrations
 */
void DebugSpecificCameraParameters(const std::vector<CameraCalib> &calibs, const std::vector<CameraMode> &modes)
{
	for (int c = 0; c < calibs.size(); c++)
	{
		const CameraCalib &calib = calibs[c];
		float fovH = (float)getFoVH(calib, modes[c]), fovV = (float)getFoVV(calib, modes[c]), fovD = (float)getFoVD(calib, modes[c]);
		float effH = (float)getEffectiveFoVH(calib, modes[c]), effV = (float)getEffectiveFoVV(calib, modes[c]), effD = (float)getEffectiveFoVD(calib, modes[c]);
		float distH = (effH/fovH - 1) * 100, distV = (effV/fovV - 1) * 100, distD = (effD/fovD - 1) * 100;
		LOGCL("    Cam %d: FoV (H,V,D) (%f, %f, %f); P (%f, %f); D (%f, %f, %f, %f, %f); Distortion (%.2f%%, %.2f%%, %.2f%%); Effective FoV: (%.2fdg, %.2fdg, %.2fdg)\n",
			c, fovH, fovV, fovD,
			calib.principalPoint.x(), calib.principalPoint.y(),
			calib.distortion.k1, calib.distortion.k2, calib.distortion.p1, calib.distortion.p2, calib.distortion.k3,
			distH, distV, distD, effH, effV, effD);
	}
}

/**
 * Log the parameters and inferred properties of the camera calibrations
 */
void DebugCameraParameters(const std::vector<CameraCalib> &calibs)
{
	for (int c = 0; c < calibs.size(); c++)
	{
		const CameraCalib &calib = calibs[c];
		float fovH = (float)getFoVH(calib);
		float effH = (float)getEffectiveFoVH(calib);
		float distH = (effH/fovH - 1) * 100;
		LOGC(LDebug, "    Cam %d: FoV (H) (%f); P (%f, %f); D (%f, %f, %f, %f, %f); Distortion (%.2f%%); Effective FoV: (%.2fdg)\n",
			calib.id, fovH,
			calib.principalPoint.x(), calib.principalPoint.y(),
			calib.distortion.k1, calib.distortion.k2, calib.distortion.p1, calib.distortion.p2, calib.distortion.k3,
			distH, effH);
	}
}

/**
 * Update the error maps in pipeline with errors calculated from the given calibs and data
 */
void UpdateErrorMaps(PipelineState &pipeline, const ObsData &data, const std::vector<CameraCalib> &calibs)
{
	std::vector<CameraErrorMaps*> errorMaps(calibs.size());
	std::vector<SynchronisedS<CameraErrorMaps>::LockedPtr> errorMapLocks;
	for (int c = 0; c < calibs.size(); c++)
	{
		if (calibs[c].invalid()) continue;
		std::shared_ptr<CameraPipeline> &cam = pipeline.cameras[calibs[c].index];
		cam->errorVisDirty = false;
		auto err_lock = cam->errorVis.contextualLock();
		err_lock->mapSize = Eigen::Vector2i(cam->mode.widthPx/32, cam->mode.heightPx/32);
		// Stope pointer to error maps
		errorMaps[c] = &*err_lock;
		// Keep lock alive until error maps are not used anymore
		errorMapLocks.push_back(std::move(err_lock));
	}
	pipeline.pointCalib.state.errors = updateCameraErrorMaps(data, calibs, errorMaps);
	for (int c = 0; c < calibs.size(); c++)
	{
		if (calibs[c].invalid()) continue;
		pipeline.cameras[calibs[c].index]->errorVisDirty = true;
	}
}

void UpdateErrorFromObservations(PipelineState &pipeline, bool errorMaps)
{
	std::vector<CameraCalib> calibs = pipeline.getCalibs();
	// TODO: Nothing in the following code actually handles invalid calibrations - not serious, still needs fixing

	ObsData data;
	{ // Copy data
		auto obs_lock = pipeline.seqDatabase.contextualRLock();
		addTriangulatableObservations(data.points, obs_lock->markers, 0, obs_lock->lastRecordedFrame);
	}

	if (errorMaps)
	{
		ScopedLogLevel scopedLogLevel(LDebug);
		UpdateErrorMaps(pipeline, data, calibs);
	}

	{
		ScopedLogLevel scopedLogLevel(LInfo);
		pipeline.pointCalib.state.errors = updateReprojectionErrors(data, calibs);
	}
}

void UpdateCalibrationRelations(const PipelineState &pipeline, CameraSystemCalibration &calibration, OptErrorRes error, unsigned int num)
{
	for (int i = 1; i < pipeline.cameras.size(); i++)
	{
		if (pipeline.cameras[i]->calib.invalid())
			continue;
		for (int j = 0; j < i; j++)
		{
			if (pipeline.cameras[j]->calib.invalid())
				continue;
			// Determine fundamental matrix from calibration
			FundamentalMatrix FM = {};
			FM.matrix = calculateFundamentalMatrix<float>(pipeline.cameras[i]->calib, pipeline.cameras[j]->calib);
			FM.precalculated = true;
			FM.stats.avg = error.mean;
			FM.stats.num = num;
			FM.stats.max = error.max;
			FM.stats.M2 = error.stdDev*error.stdDev*num;
			calibration.relations.setFundamentalMatrix(i, j, FM);
		}
	}
}

static void UpdateCalibrationRelation(CameraSystemCalibration &calibration, const SequenceData &sequences, const CameraCalib &calibA, const CameraCalib &calibB)
{
	assert(calibA.index > calibB.index);
	assert(calibA.index >= 0);
	assert(calibB.index >= 0);

	// TODO: Respect start (and end?) frame for which calibration is valid
	// E.g. might be triggered after a disruption, at which point prior samples should be ignored

	// Determine fundamental matrix from calibration
	FundamentalMatrix FM = {};
	FM.matrix = calculateFundamentalMatrix<float>(calibA, calibB);
	FM.precalculated = true;

	// Calculate full error stats
	handleSharedObservations(sequences.markers, 0, sequences.lastRecordedFrame, calibA.index, calibB.index,
		[&FM](int marker, auto itA, auto itB, int frame, int length) {
			for (int i = 0; i < length; ++i, ++itA, ++itB)
			{
				float error = itB->homogeneous().transpose() * FM.matrix * itA->homogeneous();
				FM.stats.update(std::abs(error));
			}
		});

	// Set as precalculated FM
	calibration.relations.setFundamentalMatrix(calibA.index, calibB.index, FM);
}

void UpdateCalibrationRelations(const PipelineState &pipeline, CameraSystemCalibration &calibration, const SequenceData &sequences)
{
	for (int i = 1; i < pipeline.cameras.size(); i++)
	{
		if (pipeline.cameras[i]->calib.invalid())
			continue;
		for (int j = 0; j < i; j++)
		{
			if (pipeline.cameras[j]->calib.invalid())
				continue;
			UpdateCalibrationRelation(calibration, sequences, pipeline.cameras[i]->calib, pipeline.cameras[j]->calib);
		}
	}
}

void UpdateCalibrationRelations(const PipelineState &pipeline, CameraSystemCalibration &calibration, const SequenceData &sequences, int camIndex)
{
	if (pipeline.cameras[camIndex]->calib.invalid())
		return;
	for (int j = 0; j < camIndex; j++)
	{
		if (pipeline.cameras[j]->calib.invalid())
			continue;
		UpdateCalibrationRelation(calibration, sequences, pipeline.cameras[camIndex]->calib, pipeline.cameras[j]->calib);
	}
	for (int j = camIndex+1; j < pipeline.cameras.size(); j++)
	{
		if (pipeline.cameras[j]->calib.invalid())
			continue;
		UpdateCalibrationRelation(calibration, sequences, pipeline.cameras[j]->calib, pipeline.cameras[camIndex]->calib);
	}
}

/**
 * Determine affine transformation between current calibration and ground truth from simulation setup
 * and apply it to be able to compare to ground truth
 * Returns remaining errors (positional in mm, angular in degrees)
 */
std::pair<CVScalar,CVScalar> AlignWithGT(const PipelineState &pipeline, std::vector<CameraCalib> &calibs)
{
	// Collect camera positions as corresponding point clouds
	Eigen::Matrix<CVScalar,Eigen::Dynamic,3> trMat(calibs.size(), 3);
	Eigen::Matrix<CVScalar,3,Eigen::Dynamic> mkMat(3, calibs.size());
	for (int c = 0; c < calibs.size(); c++)
	{
		const std::shared_ptr<CameraPipeline> &camera = pipeline.cameras[calibs[c].index];
		trMat.row(c) = camera->simulation.calib.transform.translation();
		mkMat.col(c) = calibs[c].transform.translation();
	}

	// Determine affine transformation between them
	Eigen::Transform<CVScalar,3,Eigen::Affine> transformGT = kabsch<CVScalar,Eigen::Affine>(trMat, mkMat);

	// Apply to calibration
	for (int c = 0; c < calibs.size(); c++)
	{
		calibs[c].transform.linear() = transformGT.rotation() * calibs[c].transform.linear();
		calibs[c].transform.translation() = transformGT * calibs[c].transform.translation();
		calibs[c].UpdateDerived();
	}

	// Calculate errors between corrected calibration and ground truth
	std::pair<CVScalar,CVScalar> camErrors;
	for (int c = 0; c < calibs.size(); c++)
	{	
		const std::shared_ptr<CameraPipeline> &camera = pipeline.cameras[calibs[c].index];
		std::pair<CVScalar,CVScalar> poseError = calculatePoseError(camera->simulation.calib.transform, calibs[c].transform);
		camErrors.first += poseError.first;
		camErrors.second += poseError.second;
	}
	camErrors.first /= calibs.size();
	camErrors.second /= calibs.size();

	return camErrors;
}