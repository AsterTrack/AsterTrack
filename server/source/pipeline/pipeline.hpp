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

#ifndef PIPELINE_H
#define PIPELINE_H

#include "frameRecord.hpp"

#include "pipeline/parameters.hpp"

#include "calib/camera_system.hpp" // CameraSystemCalibration
#include "calib/optimisation.hpp" // OptimisationOptions, CameraErrorMaps

#include "point/parameters.hpp" // SequenceParameters
#include "point/sequence_data.hpp" // SequenceData
#include "point/tracking3D.hpp" // TrackedMarker
#include "point/triangulation.hpp" // TriangulatedPoint

#include "target/target.hpp" // TargetTemplate3D
#include "target/tracking3D.hpp" // TrackedTargetFiltered

#include "calib_point/parameters.hpp" // PointCalibParameters
#include "calib_target/parameters.hpp" // TargetCalibParameters
#include "calib_target/aquisition.hpp" // TargetViewAquisition
#include "calib_target/assembly.hpp" // TargetView, TargetAssemblyStage, TargetAssemblyStageID

#include "simulation.hpp" // SimulationState

#include "util/blocked_vector.hpp"
#include "util/eigendef.hpp"
#include "util/synchronised.hpp"
#include "util/threading.hpp"

#include <vector>
#include <list>

/**
 * Main processing pipeline for tracking and calibration
 * Currently split in explicit phases, might be organically merged in the future
 */


/* Structures */


enum PipelinePhase
{
	PHASE_Idle,
	PHASE_Automatic,
	PHASE_Calibration_Point,
	PHASE_Calibration_Target,
	PHASE_Tracking
};

/**
 * Host-side state of a TrackingCamera
 */
struct CameraPipeline
{
	CameraID id = CAMERA_ID_NONE;
	int index = -1;
	CameraCalib calib = {};
	CameraMode mode = {};

	// Error data of calibration
	std::atomic<bool> errorVisDirty = { false };
	SynchronisedS<CameraErrorMaps> errorVis = {};

	// Ground-truth data generated in simulation mode
	struct {
		CameraCalib calib = {};
	} simulation = {};

	CameraPipeline() {}
	CameraPipeline(CameraID ID, int Index) : id(ID), index(Index) {}
};


/**
 * State of algorithms of different phases
 */
struct PipelineState
{
	recursive_shared_mutex pipelineLock;

	// This is only a hint and UI state
	std::atomic<enum PipelinePhase> phase = PHASE_Idle;

	// Cameras
	std::vector<std::shared_ptr<CameraPipeline>> cameras;
	inline std::vector<CameraCalib> getCalibs() const
	{
		std::vector<CameraCalib> calibs(cameras.size());
		for (auto &camera : cameras)
			calibs[camera->index] = camera->calib;
		return calibs;
	}

	// Management of calibration of the camera system as a whole
	SynchronisedS<CameraSystemCalibration> calibration;

	// Frames
	std::atomic<long> frameNum = -1;
	BlockedQueue<std::shared_ptr<FrameRecord>> frameRecords;
	bool keepFrameRecordsDefault = true, keepFrameRecords = true, keepFrameImages = true;
	// TODO: Periodically cull_front+delete_culled to free old frame records
	// Various calibration routines might need access to older frame records in the queue
	// So targetViews and all of TargetAssembly needs to be respected and/or similarly managed
	// Same for obsDatabase if there is any expectation of e.g. being able to access the blob value of any one sample
	// Recent ones might still be referenced for rendering and detection

	// Recorded 2D sequences, used for both point calib and target calib
	bool recordSequences = true;
	SynchronisedS<SequenceData> seqDatabase = {};
	SequenceParameters sequenceParams = {}; // Parameters for 2D sequence algorithm

	// Shared database of observations (both points and targets) for continuous optimisation and related algorithms
	Synchronised<ObsData> obsDatabase;

	// Params for tracking and related algorithms
	TrackingParameters params = {};

	// Tracking pipeline state
	struct {
		TimePoint_t lastFrameTime; // Time of last tracked frame for timestep calculation
		// All stored target calibrations
		// TODO: This is the actual store, though pipeline resets needs this, it might be better stored in the main server
		std::vector<TargetTemplate3D> targetTemplates3D;
		// Currently triangulated points
		std::vector<Eigen::Vector3f> points3D; // points of triangulations3D
		std::vector<TriangulatedPoint> triangulations3D; // Likely to be real markers
		std::vector<TriangulatedPoint> discarded3D; // Unlikely to be real markers
		// Currently tracked objects
		std::list<TrackedTargetFiltered> trackedTargets;
		std::list<TrackedMarker<float>> trackedMarkers;
		// Targets not currently tracked
		std::list<TargetTemplate3D const *> unusedTargets;
		std::list<std::pair<TargetTemplate3D const *,int>> lostTargets;
		// Asynchronous Detections
		bool asyncDetection;
		int asyncDetectTargetID = 0;
		std::stop_source asyncDetectionStop, syncDetectionStop;
		// Add information which cluster async detection handles to interrupt it should that cluster disappear or be detected
		// Currently we only check for which target ID is being attempted to be detected, to abort when that target has been found 
	} tracking = {};

	// Point calibration state
	struct {
		PointCalibParameters params;
		// Calibration State (both reconstruction and optimisation)
		bool planned;
		ThreadControl control;
		struct
		{
			int typeFlags;
			OptimisationOptions options = OptimisationOptions(false, true, true, true, true);
			int maxSteps = 20;
		} settings;
		struct
		{
			int numSteps = 0;
			OptErrorRes errors = {};
			int lastStopCode;
			bool complete = false;
		} state;
		// Room parameters
		std::vector<PointCalibration<double>> floorPoints;
		float distance12 = 50.0f; // In mm
		bool normaliseRoom, calibrateFloor;
	} pointCalib = {};

	// Target calibration state
	struct {
		TargetCalibParameters params;
		// State for heuristic target view aquisition
		Synchronised<TargetViewAquisition> aquisition;
		// Acquired target views and their reconstruction
		Synchronised<std::vector<std::shared_ptr<TargetView>>> views;
		// State and stages of assembly of views into final target
		std::shared_ptr<TargetView> baseView = nullptr;
		struct
		{
			bool planned;
			ThreadControl control;
			struct
			{
				bool followAlgorithm = true;
				// If not, follow custom instructions & parameters
				std::vector<TargetAssemblyStageID> instructions;
				int maxSteps;
				float tolerances;
			} settings;
			struct
			{
				std::string currentStage;
				bool optimising = false;
				int numSteps = 0, maxSteps = 0;
				OptErrorRes errors = {}; // Errors after last proper oprimisation
			} state;
		} assembly;
		Synchronised<std::vector<std::shared_ptr<TargetAssemblyStage>>> assemblyStages;
	} targetCalib = {};

	// Simulation state
	bool isSimulationMode;
	SynchronisedS<SimulationState> simulation = {};
};


/* Functions */

/**
 * Init/Reset pipeline subsystems
 */
void ResetPipeline(PipelineState &pipeline);

// Calibration
void InitPointCalibration(PipelineState &pipeline);
void UpdatePointCalibration(PipelineState &pipeline, std::vector<CameraPipeline*> &cameras);

// Target Calibration
void InitTargetCalibration(PipelineState &pipeline);
void UpdateTargetCalibration(PipelineState &pipeline, std::vector<CameraPipeline*> &cameras, unsigned int frameNum);

// General Tracking
void InitTrackingPipeline(PipelineState &pipeline);
void UpdateTrackingPipeline(PipelineState &pipeline, std::vector<CameraPipeline*> &cameras, std::shared_ptr<FrameRecord> &frame);

/**
 * Handles a new frame and returns it once processed
 */
void ProcessFrame(PipelineState &pipeline, std::shared_ptr<FrameRecord> newFrame);

void PreprocessCameraData(const CameraCalib &calib, CameraFrameRecord &record);

void PreprocessFrame(const PipelineState &pipeline, FrameRecord &record);

/* For simulation/replay to jump to a specific frame - caller has to make sure no more queued frames will be processed */
void AdoptFrameRecordState(PipelineState &pipeline, const FrameRecord &frameRecord);

/* General functions */

/**
 * Log the parameters and inferred properties of the camera calibrations
 */
void DebugSpecificCameraParameters(const std::vector<CameraCalib> &calibs, const std::vector<CameraMode> &modes);

/**
 * Log the parameters and inferred properties of the camera calibrations
 */
void DebugCameraParameters(const std::vector<CameraCalib> &calibs);

/**
 * Update the error maps for visualisation across each camera frame
 */
void updateErrorMaps(PipelineState &pipeline, const ObsData &data, const std::vector<CameraCalib> &cameras);

void UpdateErrorFromObservations(PipelineState &pipeline, bool errorMaps = true);

void UpdateCalibrationRelations(const PipelineState &pipeline, CameraSystemCalibration &calibration, OptErrorRes error, unsigned int num);

void UpdateCalibrationRelations(const PipelineState &pipeline, CameraSystemCalibration &calibration, const SequenceData &sequences);

void UpdateCalibrationRelations(const PipelineState &pipeline, CameraSystemCalibration &calibration, const SequenceData &sequences, int camIndex);

/**
 * Determine affine transformation between current calibration and ground truth from simulation setup
 * and apply it to be able to compare to ground truth
 * Returns remaining errors (positional in mm, angular in degrees)
 */
std::pair<CVScalar,CVScalar> AlignWithGT(const PipelineState &pipeline, std::vector<CameraCalib> &calibs);

#endif // PIPELINE_H