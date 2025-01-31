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

#include "ui/shared.hpp" // Signals

#include "calib_target/parameters.hpp"
#include "calib_target/aquisition.hpp"
#include "calib_target/reconstruction.hpp"
#include "calib_target/assembly.hpp"

#include "calib/obs_data.inl"
#include "target/detection3D.hpp"
#include "point/sequence_data.inl"

#include "util/log.hpp"
#include "util/matching.hpp"

#include "scope_guard/scope_guard.hpp"

#include <numeric>

static void ThreadCalibrationTargetView(std::stop_token stopToken, PipelineState *pipeline, std::vector<CameraPipeline*> cameras, std::shared_ptr<TargetView> viewPtr);
static void ThreadCalibrationTarget(std::stop_token stopToken, PipelineState *pipeline, std::vector<CameraPipeline*> cameras);

// ----------------------------------------------------------------------------
// Target Calibration
// ----------------------------------------------------------------------------

void InitTargetCalibration(PipelineState &pipeline)
{
	auto &tgtCalib = pipeline.targetCalib;

	// Request all calibration threads to stop if they are still running
	tgtCalib.assembly.control.stop_source.request_stop();
	for (auto &view : *tgtCalib.views.contextualRLock())
		view->control.stop_source.request_stop();
	// Wait for threads to stop - not entirely necessary, but else threads will never be deleted
	tgtCalib.assembly.control.stop();
	// Destructor of views will join and thus block
	// TODO: Make thread stopping async to prevent UI blocking while waiting for an optimisation to finish
	// Threads should already be instructed to not write, but need to make extra sure if it will be async
	// since then pipeline might already be used again if a thread took long to finish
	// which they do rn as optimisation step cannot be interrupted
	// Same applies to point calibration of course

	// Init/Clean the rest
	tgtCalib.assembly.planned = false;
	tgtCalib.assembly.settings = {};
	tgtCalib.assembly.state = {};
	tgtCalib.assemblyStages.contextualLock()->clear();
	tgtCalib.views.contextualLock()->clear();
	tgtCalib.aquisition.contextualLock()->reset();
}

void FinaliseTargetCalibration(PipelineState &pipeline)
{
	// TODO: Proper target transform calibration step (centering, alignment)
	// Not super important for tracking but nice-to-have atleast a good default
	// Related: Currently, optimisation may shift the transform without impacting calibration, a repeatable "normalisation" step would fix that

	// Find max occupied marker id (excluding testing markers because they are negative)
	/*int maxID = -1;
	for (int i = 0; i < pipeline.tracking.targetTemplates3D.size(); i++)
		maxID = std::max(maxID, pipeline.tracking.targetTemplates3D[i].id);

	// Add marker with new ID to templates
	pipeline.tracking.targetTemplates3D.push_back({});
	TargetTemplate3D &marker = pipeline.tracking.targetTemplates3D[pipeline.tracking.targetTemplates3D.size()-1];
	marker.id = maxID+1;
	marker.label = std::string("Target ID ") + (char)((int)'0' + marker.id);
	marker.points.reserve(cluster.targetMarkers.size());
	for (int i = 0; i < cluster.targetMarkers.size(); i++)
		marker.points.push_back(cluster.targetMarkers[i].calib.pos);
	marker.updateLookupTables();
	LOG(LTargetCalib, LDebug, "Saved marker with %d points as id %d!\n", (int)marker.points.size(), marker.id); */

}

void UpdateTargetCalibration(PipelineState &pipeline, std::vector<CameraPipeline*> &cameras, unsigned int frameNum)
{
	auto &tgtCalib = pipeline.targetCalib;

	if (pipeline.recordSequences)
	{
		auto aquisition_lock = tgtCalib.aquisition.contextualLock();

		int stableFrame = frameNum - stableSequenceDelay;

		{ // Update target view aquisition
			auto obs_lock = pipeline.seqDatabase.contextualRLock();

			// Update preliminary stats, for vis mostly
			updateTargetViewAquisitionStats(*obs_lock, *aquisition_lock, frameNum, 0);

			// Sequence has a few frames delay until it becomes stable
			if (aquisition_lock->markerCount.size() >= stableSequenceDelay)
			{
				// Re-determine stats of stable sequence
				updateTargetViewAquisitionStats(*obs_lock, *aquisition_lock, stableFrame, stableSequenceDelay);

				// Update current range using stats
				bool finalise = updateTargetViewAquisition(*aquisition_lock, stableFrame, stableSequenceDelay, tgtCalib.params.aquisition);

				TargetViewRange newRange;
				// Finalise current range
				if (finalise && finaliseTargetViewAquisitionRange(*obs_lock, *aquisition_lock, tgtCalib.params.aquisition, newRange))
				{
					auto views_lock = tgtCalib.views.contextualLock();
					auto view = std::make_shared<TargetView>();
					view->id = views_lock->empty()? 0 : views_lock->back()->id+1;
					view->beginFrame = newRange.beginFrame;
					view->endFrame = newRange.endFrame;
					view->stats = newRange.stats;
					view->targetTemplate.initialise(newRange.target.markers);
					*view->target.contextualLock() = std::move(newRange.target);
					if (pipeline.isSimulationMode)
					{
						view->simulation.targetGT = pipeline.simulation.contextualRLock()->getPrimary().target;
					}
					{ // Reconstruct and optimise view immediately
						view->settings.typeFlags = 0b11;
						view->settings.maxSteps = pipeline.targetCalib.params.view.initialOptLimit;
						view->settings.tolerances = pipeline.targetCalib.params.view.initialOptTolerance;
						view->planned = true;
					}
					views_lock->push_back(std::move(view));
				}
			}
		}

		// TODO: Use aquisition.currentValue as feedback to user (e.g. sound pitch)
		// It is delayed by [stableSequenceDelay] frames though
		// Consider similar multimodal feedback to user for point calibration 
	}

	for (auto &view : *tgtCalib.views.contextualRLock())
	{
		if (view->control.running())
		{
			if (view->control.finished)
			{
				view->control.stop();
				SignalPipelineUpdate();
			}
		}
		else if (view->planned)
		{
			view->planned = false;
			view->control.init();
			view->control.thread = new std::thread(ThreadCalibrationTargetView, view->control.stop_source.get_token(), &pipeline, cameras, view);
			SignalPipelineUpdate();
		}
	}

	{
		if (tgtCalib.assembly.control.running())
		{
			if (tgtCalib.assembly.control.finished)
			{
				tgtCalib.assembly.control.stop();
				SignalPipelineUpdate();
			}
		}
		else if (tgtCalib.assembly.planned)
		{
			tgtCalib.assembly.planned = false;
			tgtCalib.assembly.control.init();
			tgtCalib.assembly.control.thread = new std::thread(ThreadCalibrationTarget, tgtCalib.assembly.control.stop_source.get_token(), &pipeline, cameras);
			SignalPipelineUpdate();
		}
	}
}

// Update data of tgtGT to newly projected GT data, used for debugging in simulation
static void makeGTTarget(const SimulationState &simulation, const SequenceData &sequences, const std::vector<CameraCalib> &calibs, ObsTarget &targetGT, const TargetTemplate3D *templateGT)
{
	if (!templateGT) return;
	// Set marker positions to most likely GT candidate
	for (auto m : targetGT.markerMap)
	{
		auto gtMarker = sequences.markers[m.first].resolveGTMarker();
		if (gtMarker.first < 0)
		{
			LOGC(LDarn, "Marker %d had unknown GT!\n", m.first);
			continue;
		}
		if (gtMarker.second < 0.99f)
		{
			LOGC(LDebug, "Marker %d had uncertain GT, best is %d with %f%% certainty!\n",
				m.first, gtMarker.first, gtMarker.second*100);
		}
		targetGT.markers[m.second] = templateGT->markers[gtMarker.first].pos;
	}
	// Take GT frame poses and overwrite observations with GT
	for (auto &frame : targetGT.frames)
	{
		frame.pose = simulation.framePoses[frame.frame];

		for (auto &samples : frame.samples)
		{
			Eigen::Vector3f gtMarker = targetGT.markers[targetGT.markerMap.at(samples.marker)];
			Eigen::Vector2f gtPoint = projectPoint2D(calibs[samples.camera].camera, frame.pose * gtMarker);
			samples.point = distortPointUnstable(calibs[samples.camera], gtPoint, 1000, 0.001f*PixelSize);
		}
	}
}

static void ThreadCalibrationTargetView(std::stop_token stopToken, PipelineState *pipeline, std::vector<CameraPipeline*> cameras, std::shared_ptr<TargetView> viewPtr)
{
	const auto exitNotifier = sg::make_scope_guard([&]() noexcept { viewPtr->control.finished = true; });

	ScopedLogCategory scopedLogCategory(LTargetCalib, true);
	ScopedLogContext scopedLogContext(viewPtr->beginFrame); // Need something visible in the UI
	ScopedLogLevel scopedLogLevel(LDebug);

	LOGC(LDebug, "=======================\n");

	// Copy data
	auto settings = viewPtr->settings;
	TargetCalibParameters params = pipeline->targetCalib.params;
	// Copy camera calibration
	std::vector<CameraCalib> calibs(cameras.size());
	for (int c = 0; c < cameras.size(); c++)
		calibs[c] = cameras[c]->calib;
	// Copy OptTarget
	ObsData obsData;
	obsData.targets.push_back(*viewPtr->target.contextualRLock());
	bool debugIterations = viewPtr->debugIterations;

	auto updateTargetView = [&]()
	{
		if (viewPtr.use_count() == 1)
		{
			LOGC(LDebug, "Source target view has been deleted during calibration!\n");
			LOGC(LDebug, "=======================\n");
			return false;
		}
		viewPtr->state.calibrated = true;
		if (viewPtr->state.errors.rmse < 0.5*PixelSize && viewPtr->state.errors.max < 1)
			viewPtr->selected = true;
		ObsTarget &target = obsData.targets.front();
		if (target.markers.empty() || target.frames.empty())
		{
			LOGC(LInfo, "Target view has lost all data during optimisation! Deleting!\n");
			auto views_lock = pipeline->targetCalib.views.contextualLock();
			auto it = std::find(views_lock->begin(), views_lock->end(), viewPtr);
			if (it != views_lock->end())
				views_lock->erase(it);
			else
				LOGC(LWarn, "Failed to remove target view from view list!\n");
			if (viewPtr.use_count() > 1) // Can't delete e.g. visState.targetView
				LOGC(LWarn, "Target view is still referenced!\n");
			LOGC(LDebug, "=======================\n");
			return false;
		}
		viewPtr->targetTemplate = TargetTemplate3D(finaliseTargetMarkers(calibs, target, params.post));
		*viewPtr->target.contextualLock() = target;
		viewPtr->stats.frameCount = target.frames.size();
		viewPtr->stats.markerCount = target.markers.size();
		viewPtr->stats.sampleCount = target.totalSamples;
		debugIterations = debugIterations && viewPtr->debugIterations;
		if (debugIterations)
			viewPtr->iterationStates.emplace_back(viewPtr->state.errors, target);
		SignalPipelineUpdate();
		return true;
	};

	// Build a GT target
	ObsTarget targetGT = obsData.targets.front();
	if (pipeline->isSimulationMode)
		makeGTTarget(*pipeline->simulation.contextualRLock(), *pipeline->seqDatabase.contextualRLock(), calibs, targetGT, viewPtr->simulation.targetGT);

	auto alignWithGT = [](ObsTarget &target, const ObsTarget &tgtGT)
	{
		// Adjust tgt to align with tgtGT
		Eigen::Isometry3f GTpose = getTransformToGT(target, tgtGT);
		for (auto &marker : target.markers)
			marker = GTpose * marker;
		GTpose = GTpose.inverse();
		for (auto &frame : target.frames)
			frame.pose = frame.pose * GTpose;
	};
	auto normaliseTarget = [](ObsTarget &target)
	{
		// Adjust targets CoM
		Eigen::Vector3f com = Eigen::Vector3f::Zero();
		for (auto &marker : target.markers)
			com += marker;
		com /= target.markers.size();
		for (auto &marker : target.markers)
			marker -= com;
		for (auto &frame : target.frames)
			frame.pose.translation() += frame.pose.rotation() * com;
	};

	if (settings.typeFlags & 0b01)
	{
		LOGC(LDebug, "== Reconstructing Target View:\n");

		int maxIteration = 100, minAbortIteration = 100, correctIteration = 20, dropIteration = 100;
	#ifndef NDEBUG // Debug mode is too slow, use fewer iterations
		maxIteration = 20;
		minAbortIteration = 20;
		correctIteration = 20;
		dropIteration = 20;
	#endif

		// Pass on a Ground-Truth target in simulation
		ObsTarget *tgtGTPtr = nullptr;
		if (pipeline->isSimulationMode)
			tgtGTPtr = &targetGT;

		bool success = reconstructTarget(calibs, obsData.targets.front(),
			maxIteration, minAbortIteration, correctIteration, dropIteration,
			tgtGTPtr);

		if (!success)
		{
			LOGC(LWarn, "Failed to reconstruct target motion and structure!\n");
			LOGC(LDebug, "=======================\n");
			return;
		}

		if (pipeline->isSimulationMode)
		{
			debugTargetResults(obsData.targets.front(), targetGT);
			alignWithGT(obsData.targets.front(), targetGT);
		}
		else
		{
			normaliseTarget(obsData.targets.front());
		}

		OptErrorRes error = getTargetErrorDist(calibs, obsData.targets.front());
		LOGC(LDebug, "Target reprojection rmse of %fpx, with %d samples\n",
			error.rmse*PixelFactor, (int)obsData.targets.front().totalSamples);
		viewPtr->settings.typeFlags &= ~0b01;
	}

	// Calculate error and update source on main thread with new results
	viewPtr->state.errors = updateReprojectionErrors(obsData, calibs);
	if (!updateTargetView())
		return;
	if (stopToken.stop_requested())
		return;

	if (settings.typeFlags & 0b10)
	{
		LOGC(LDebug, "== Optimising Target View:\n");

		OptimisationOptions options(true, false, false, false, false);

		auto lastIt = pclock::now();
		bool continueOptimisation = false;

		auto itUpdate = [&](OptErrorRes errors){
			ScopedLogLevel scopedLogLevel(LTrace);

			viewPtr->state.errors = errors;
			viewPtr->state.numSteps++;

			bool hasNaN = std::isnan(errors.mean) || std::isnan(errors.stdDev) || std::isnan(errors.max);
			if (hasNaN)
				LOGC(LWarn, "    Current errors has NaNs after optimisation step %d: (%f, %f, %f)\n",
					viewPtr->state.numSteps, errors.mean, errors.stdDev, errors.max);

			LOGC(LDebug, "   Optimisation step took %.2fms!\n", dtMS(lastIt, pclock::now()));
			lastIt = pclock::now();

			// Debug step
			if (pipeline->isSimulationMode)
			{
				debugTargetResults(obsData.targets.front(), targetGT);
				alignWithGT(obsData.targets.front(), targetGT);
			}
			else
			{
				normaliseTarget(obsData.targets.front());
			}

			// Re-calculate final error to update optData.targets.frame.error
			viewPtr->state.errors = errors = updateReprojectionErrors(obsData, calibs);

			if (!updateTargetView())
				return false;

			// Check if we want to interrupt optimisation to merge markers or delete frames
			LOGC(LDebug, "Reprojection rmse of %fpx, with %d samples\n",
				errors.rmse*PixelFactor, (int)obsData.targets.front().totalSamples);

			bool filtered = false;
			if (errors.max > errors.mean + params.view.outlierSigmas.trigger*errors.stdDev)
			{
				filtered = determineTargetOutliers(calibs, obsData.targets.front(), SigmaToErrors(params.view.outlierSigmas, errors), params.aquisition);

				if (obsData.targets.front().frames.empty() || obsData.targets.front().markers.empty())
				{
					LOGC(LWarn, "Removed all data during outlier determination! %d frames and %d markers left!",
						(int)obsData.targets.front().frames.size(), (int)obsData.targets.front().markers.size());
					return false;
				}
			}

			if (filtered)
			{
				continueOptimisation = !stopToken.stop_requested() && viewPtr->state.numSteps < settings.maxSteps && !hasNaN;
				return false;
			}

			return !stopToken.stop_requested() && viewPtr->state.numSteps < settings.maxSteps && !hasNaN;
		};

		do
		{
			continueOptimisation = false;
			viewPtr->state.lastStopCode = optimiseTargets(obsData, calibs, itUpdate, settings.tolerances);
		}
		while (continueOptimisation && viewPtr->state.lastStopCode < 0);
		LOGC(LDebug, "%s", getStopCodeText(viewPtr->state.lastStopCode));

		viewPtr->state.complete = !stopToken.stop_requested() && viewPtr->state.numSteps < settings.maxSteps;
		LOGC(LDebug, "Calibration finished with %d/%d steps, was%s stopped, is%s complete\n",
			viewPtr->state.numSteps, settings.maxSteps, stopToken.stop_requested()? "" : " not", viewPtr->state.complete? "" : " not");
		viewPtr->settings.typeFlags &= ~0b10;
	}

	if (!updateTargetView())
		return;

	if (!stopToken.stop_requested()) LOGC(LDebug, "== Finished Calibrating Target View!\n")
	else LOGC(LDebug, "== Aborted Target View Calibration!\n")
}

static OptErrorRes optimiseTargetDataLoop(ObsTarget &target, const std::vector<CameraCalib> &calibs,
	const BlockedQueue<std::shared_ptr<FrameRecord>> &frameRecords, const std::vector<MarkerSequences> &observations,
	SubsampleTargetParameters params, TargetAquisitionParameters aquisition,
	OptimisationOptions &options, int maxSteps, float tolerances, TargetOutlierErrors outliers,
	std::stop_token &stop, std::function<void(OptErrorRes)> update = [](OptErrorRes){})
{
	ObsData obsData = {};
	OptErrorRes startErrors = getTargetErrorDist(calibs, target);
	determineTargetOutliers(calibs, target, SigmaToErrors(outliers, startErrors), aquisition);
	if (params.resampleInterval > 0)
	{
		ObsTarget subsampled = subsampleTargetObservations(frameRecords, observations, target, params);
		LOGC(LDebug, "    Starting optimisation with %fpx for %d total samples, subsampled to %fpx for %d samples",
			getTargetErrorDist(calibs, target).rmse*PixelFactor, target.totalSamples,
			getTargetErrorDist(calibs, subsampled).rmse*PixelFactor, subsampled.totalSamples);
		obsData.targets.push_back(std::move(subsampled));
	}
	else
	{
		obsData.targets.push_back(std::move(target));
	}

	auto lastIt = pclock::now();
	bool continueOptimisation = false;
	int steps = 0;
	int lastStopCode;
	OptErrorRes lastErrors;

	auto applySubsampledChanges = [](const ObsTarget &subsampled, ObsTarget &target)
	{
		// TODO: Discard outliers found during optimisation in source data
		// Or, perhaps better since it would apply to all data, re-determine outliers
		auto frameIt = target.frames.begin();
		for (auto &frame : subsampled.frames)
		{
			while (frameIt->frame < frame.frame)
				frameIt++;
			frameIt->pose = frame.pose;
		}

		for (auto &map : subsampled.markerMap)
		{ // Just in case mapping changed
			int tgtMarker = target.markerMap[map.first];
			target.markers[tgtMarker] = subsampled.markers[map.second];
		}
	};

	auto itUpdate = [&](OptErrorRes errors){
		ScopedLogLevel scopedLogLevel(LTrace);
		steps++;
		lastErrors = errors;
		update(errors);

		bool hasNaN = std::isnan(errors.mean) || std::isnan(errors.stdDev) || std::isnan(errors.max);
		if (hasNaN)
			LOGC(LWarn, "    Current errors has NaNs after optimisation step %d: (%f, %f, %f)\n", steps, errors.mean, errors.stdDev, errors.max);

		LOGC(LDebug, "   Optimisation step took %.2fms!\n", dtMS(lastIt, pclock::now()));
		lastIt = pclock::now();

		// Check if we want to interrupt optimisation to merge markers or delete frames
		LOGC(LDebug, "Reprojection rmse of %fpx, with %d samples\n",
			errors.rmse*PixelFactor, (int)obsData.targets.front().totalSamples);

		bool filtered = false;
		if (errors.max > errors.mean + outliers.trigger*errors.stdDev)
		{
			filtered = determineTargetOutliers(calibs, obsData.targets.front(), SigmaToErrors(outliers, errors), aquisition);

			if (obsData.targets.front().frames.empty() || obsData.targets.front().markers.empty())
			{
				LOGC(LWarn, "Removed all data during outlier determination! %d frames and %d markers left!",
					(int)obsData.targets.front().frames.size(), (int)obsData.targets.front().markers.size());
				return false;
			}
		}

		bool resampled = params.resampleInterval > 0 && steps % params.resampleInterval == 0;
		if (resampled)
		{
			applySubsampledChanges(obsData.targets.front(), target);
			LOGC(LDebug, "    Optimised to %fpx for %d subsamples, total %fpx for %d samples",
				getTargetErrorDist(calibs, obsData.targets.front()).rmse*PixelFactor, obsData.targets.front().totalSamples,
				getTargetErrorDist(calibs, target).rmse*PixelFactor, target.totalSamples);
			obsData.targets.front() = subsampleTargetObservations(frameRecords, observations, target, params);
			LOGC(LDebug, "    Subsampled to %fpx for %d samples",
				getTargetErrorDist(calibs, obsData.targets.front()).rmse*PixelFactor, obsData.targets.front().totalSamples);
		}

		continueOptimisation = !stop.stop_requested() && steps < maxSteps && !hasNaN;
		if (filtered || resampled) return false; // Restart optimisation as data changed
		return continueOptimisation;
	};

	do
	{
		continueOptimisation = false;
		lastStopCode = optimiseTargets(obsData, calibs, itUpdate, tolerances);
	}
	while (continueOptimisation && lastStopCode < 0);

	if (params.resampleInterval > 0 && steps % params.resampleInterval == 0)
	{
		applySubsampledChanges(obsData.targets.front(), target);
		OptErrorRes subError = lastErrors;
		lastErrors = getTargetErrorDist(calibs, target);
		LOGC(LDebug, "    Optimised to %fpx for %d subsamples, total %fpx for %d samples",
			subError.rmse*PixelFactor, obsData.targets.front().totalSamples,
			lastErrors.rmse*PixelFactor, target.totalSamples);
	}
	else
		target = std::move(obsData.targets.front());

	if (lastStopCode > 0)
		LOGC(LInfo, "    %s", getStopCodeText(lastStopCode));
	LOGC(LInfo, "  After Optimisation to tolerance %f in %d steps: 2D reprojection rmse of %fpx for all %d samples (%d frames and %d markers)\n",
		tolerances, steps, lastErrors.rmse*PixelFactor, target.totalSamples,
		(int)target.frames.size(), (int)target.markers.size());

	return lastErrors;
}

static int findTargetAlignmentCandidates(const TargetTemplate3D &baseTemplate, const ObsTarget &target,
	const TargetAssemblyParameters &params, std::vector<TargetCandidate3D> &candidates)
{
	// Prepare interface for target detection
	std::vector<TriangulatedPoint> triPoints;
	triPoints.reserve(target.markers.size());
	for (const Eigen::Vector3f &pos : target.markers)
		triPoints.push_back(TriangulatedPoint(pos, params.alignPointError/1000, 10.0f));
	std::vector<int> triIndices(triPoints.size());
	std::iota(triIndices.begin(), triIndices.end(), 0);

	// Match markers between the two targets
	candidates.clear();
	detectTarget3D(baseTemplate, triPoints, triIndices, candidates, params.alignPointSigma, params.alignPoseSigma, false);

	std::tuple<int,int> bestCand = getBestTargetCandidate(baseTemplate, triPoints, candidates);
	return std::get<0>(bestCand);
}

static void realignTargetViewsToBase(TargetAssemblyBase &base, std::vector<std::shared_ptr<TargetView>> &targetViews,
	const std::vector<CameraCalib> &calibs, const TargetAssemblyParameters &params, std::vector<TargetAlignResults> &candidates)
{
	TargetTemplate3D trkTarget(base.target.markers);
	for (auto &targetView : targetViews)
	{
		if (!targetView->selected) continue;
		if (std::find(base.merged.begin(), base.merged.end(), targetView->id) != base.merged.end())
			continue;

		auto tgt_lock = targetView->target.contextualLock();
		LOGC(LDebug, "    Target view %d starting %d with %d frames left, %d markers!",
			targetView->id, tgt_lock->frames.front().frame, (int)tgt_lock->frames.size(), (int)tgt_lock->markers.size());

		// Try to find alignment
		std::vector<TargetCandidate3D> alignCandidates;
		int best = findTargetAlignmentCandidates(trkTarget, *tgt_lock, params, alignCandidates);
		if (best < 0)
			base.alignmentStats.erase(targetView->id);
		else
			base.alignmentStats[targetView->id] = std::make_pair(alignCandidates[best].points.size(), alignCandidates[best].MSE);
		if (best < 0 || alignCandidates[best].MSE > params.alignMaxRMSE*params.alignMaxRMSE
			|| alignCandidates[best].points.size() <= params.alignMinPoints)
		{ // Not good enough to align the view to base
			if (alignCandidates.size() > 0)
			{ // Record as unsuccessful alignment
				LOGC(LDebug, "      Failed to align target view %d to base with %d candidates, best matched %d markers with RMSE of %fmm!",
					targetView->id, (int)alignCandidates.size(), (int)alignCandidates[best].points.size(), std::sqrt(alignCandidates[best].MSE));
				candidates.emplace_back(targetView->id, false, best, std::move(alignCandidates), tgt_lock->markers, std::sqrt(alignCandidates[best].MSE));
			}
			else
			{
				LOGC(LDebug, "      Failed to align target view %d to base without any candidates!",
					targetView->id);
			}
			continue;
		}
		auto &align = alignCandidates[best];
		auto originalMarkers = tgt_lock->markers;

		// Apply alignment
		Eigen::Isometry3f alignPoseInv = align.pose.inverse();
		for (auto &frame : tgt_lock->frames)
			frame.pose = frame.pose * align.pose;
		for (auto &marker : tgt_lock->markers)
			marker = alignPoseInv * marker;

		// Record as successful alignment
		LOGC(LDebug, "      Was able to align target view %d to base with %d candidates, best matched %d markers with RMSE of %fmm!",
			targetView->id, (int)alignCandidates.size(), (int)align.points.size(), std::sqrt(align.MSE));
		candidates.emplace_back(targetView->id, true, best, std::move(alignCandidates), std::move(originalMarkers), std::sqrt(align.MSE));
	}
}

static std::pair<int,int> beginNewTargetViewMerge(TargetAssemblyBase &base,
	const std::vector<MarkerSequences> &observations, const std::vector<std::shared_ptr<TargetView>> &targetViews,
	const std::vector<CameraCalib> &calibs, const TargetAssemblyParameters &params, std::vector<TargetMergeCandidate> &candidates)
{
	auto findInitialMarkerMap = [&params](const ObsTarget &base, const ObsTarget &view)
	{
		std::map<int,int> pointMap;

		std::vector<MatchCandidates<int, MatchCandidate<>>> matchCandidates;
		matchCandidates.reserve(view.markers.size());
		MatchCandidate<> candidate;

		float maxDistSq = params.mergePointLimit*params.mergePointLimit / (1000*1000);
		float maxIncludeSq = maxDistSq*params.match.primAdvantage*params.match.primAdvantage;

		LOGC(LDebug, "           Attempting to find matches between %d base and %d view.markers markers\n", (int)base.markers.size(), (int)view.markers.size());
		for (int m = 0; m < view.markers.size(); m++)
		{
			MatchCandidates<int, MatchCandidate<>> candidates = {};
			for (int p = 0; p < base.markers.size(); p++)
			{
				candidate.value = (base.markers[p] - view.markers[m]).squaredNorm();
				if (candidate.value < maxIncludeSq)
					recordMatchCandidate(candidates, candidate).index = p;
			}
			if (candidates.matches[0].index >= 0)
			{
				if (candidates.matches[1].index >= 0)
				{
					LOGC(LTrace, "                    View Marker %d: Closest base marker %d (%.4fmm), second closest %d (%.4fmm)!\n", m,
						candidates.matches[0].index, std::sqrt(candidates.matches[0].value)*1000,
						candidates.matches[1].index, std::sqrt(candidates.matches[1].value)*1000);
				}
				else
				{
					LOGC(LTrace, "                    View Marker %d: Closest base marker %d (%.4fmm)!\n", m,
						candidates.matches[0].index, std::sqrt(candidates.matches[0].value)*1000);
				}
				candidates.context = m;
				matchCandidates.push_back(std::move(candidates));
			}
		}

		resolveMatchCandidates(matchCandidates, base.markers.size(), params.match.squared());

		// Sort by best match
		std::sort(matchCandidates.begin(), matchCandidates.end(), [](auto &a, auto &b)
		{
			return !b.matches[0].valid() || (a.matches[0].valid() && a.matches[0].value < b.matches[0].value);
		});

		// Select a number of markers to merge first
		for (int m = 0; m < matchCandidates.size(); m++)
		{
			auto &match = matchCandidates[m];
			if (!match.matches[0].valid() || match.matches[0].value > maxDistSq)
				break;
			/* if (pointMap.size() >= params.mergeMinPoints && match.matches[0].value > maxDistSq/2)
			{
				LOGC(LInfo, "               Decided to stop adding markers with %d collected, next best at %fpx RMSE (> %fpx)",
					(int)pointMap.size(), match.matches[0].value*PixelFactor, maxDistSq/2*PixelFactor);
				break;
			} */
			pointMap[match.context] = match.matches[0].index;
			LOGC(LDebug, "        Integrated marker with %fmm RMSE",
				std::sqrt(match.matches[0].value)*1000);
		}

		return pointMap;
	};

	auto mergeMappedMarkers = [&params](ObsTarget &base, const ObsTarget &view, const std::map<int,int> &markerMap)
	{
		// Merge markers if they have no equivalent yet
		for (auto &m : view.markerMap)
		{
			auto map = markerMap.find(m.second);
			if (map == markerMap.end()) continue;
			auto ex = base.markerMap.find(m.first);
			if (ex != base.markerMap.end() && ex->second != map->second)
			{ // Shouldn't happen unless a sequence was visible so long to span two adjacent target views
				LOGC(LWarn, "        Merging markers of views had sequence shared among them, but markers each assigned to them differed!");
				continue;
			}
			base.markerMap[m.first] = map->second;
		}

		LOGC(LInfo, "    Added %d markers from merged view!", (int)markerMap.size());

		// Merge frames with enough observations
		int prevFrames = base.frames.size();
		auto insIt = std::upper_bound(base.frames.begin(), base.frames.end(), view.frames.front(),
			[&](auto &a, auto &b){ return a.frame < b.frame; });
		for (auto &frame : view.frames)
		{
			if (insIt != base.frames.end() && insIt->frame <= frame.frame)
			{ // Already exists
				insIt++;
				continue;
			}
			int obsCnt = 0;
			for (auto &sample : frame.samples)
			{
				auto map = base.markerMap.find(sample.marker);
				if (map != base.markerMap.end()) obsCnt++;
			}
			if (obsCnt >= params.mergeMinFrameObs)
				insIt = base.frames.insert(insIt, frame);
		}
		LOGC(LInfo, "    Added %d frames from merged view!", (int)(base.frames.size()-prevFrames));
	};

	OptErrorRes baseError = getTargetErrorDist(calibs, base.target);
	LOGC(LDebug, "    Starting with a reprojection RMSE of %fpx for %d samples (%d frames and %d markers)\n",
		baseError.rmse*PixelFactor, base.target.totalSamples,
		(int)base.target.frames.size(), (int)base.target.markers.size());
	// Lower limit to prevent over-fitting from being a problem here
	float errorLimit = std::max(1.0f*PixelSize, baseError.rmse);

	// Get list of views we can merge in order of prior alignment results
	std::vector<std::tuple<std::shared_ptr<TargetView>, int, float>> alignedViews;
	for (auto &targetView : targetViews)
	{
		if (!targetView->selected) continue;
		if (std::find(base.merged.begin(), base.merged.end(), targetView->id) != base.merged.end()) continue;
		auto alignment = base.alignmentStats.find(targetView->id);
		if (alignment == base.alignmentStats.end()) continue;
		alignedViews.emplace_back(targetView, alignment->second.first, alignment->second.second);
	}
	std::sort(alignedViews.begin(), alignedViews.end(), [](auto &a, auto &b)
	{
		return std::get<1>(a) - std::get<2>(a) > std::get<1>(b) - std::get<2>(b);
	});

	int mergeAttempts = 0;
	for (auto &view : alignedViews)
	{
		mergeAttempts++;
		auto &targetView = std::get<0>(view);
		auto tgt_lock = targetView->target.contextualRLock();

		LOGC(LDebug, "      Target view %d starting %d with %d frames left, %d markers!",
			targetView->id, tgt_lock->frames.front().frame, (int)tgt_lock->frames.size(), (int)tgt_lock->markers.size());

		std::map<int, int> markerMap = findInitialMarkerMap(base.target, *tgt_lock);
		if (markerMap.size() < params.mergeMinPoints)
		{
			LOGC(LDebug, "      Was not able to merge view with %d frames, %d markers!",
				(int)base.target.frames.size(), (int)base.target.markers.size());
			candidates.emplace_back(targetView->id, false, std::move(markerMap), tgt_lock->markers, OptErrorRes{});
			continue;
		}

		mergeMappedMarkers(base.target, *tgt_lock, markerMap);

		// ASSERT: no duplicate frames, correct order
		long lastFrame = -1;
		for (auto &frame : base.target.frames)
		{
			assert(lastFrame < (long)frame.frame);
			lastFrame = frame.frame;
		}

		updateTargetObservations(base.target, observations);

		OptErrorRes newError = getTargetErrorDist(calibs, base.target);
		LOGC(LDebug, "      Managed to merge to %d frames, %d markers, now reprojection RMSE of %fpx!",
			(int)base.target.frames.size(), (int)base.target.markers.size(), newError.rmse*PixelFactor);

		base.merging = targetView; // new shared_ptr
		candidates.emplace_back(targetView->id, true, std::move(markerMap), tgt_lock->markers, newError);
		return std::make_pair(mergeAttempts, 1);
	}

	return std::make_pair(mergeAttempts, 0);
}

static std::shared_ptr<TargetView> selectCentralTargetView(const std::vector<std::shared_ptr<TargetView>> &views,
	const std::vector<CameraCalib> &calibs, const TargetAssemblyParameters &params)
{
	// Make working copy of selected target views to consider
	std::vector<std::shared_ptr<TargetView>> targetViews;
	for (auto &viewPtr : views)
	{
		if (viewPtr->selected)
			targetViews.push_back(viewPtr); // new shared_ptr
	}
	if (targetViews.empty())
	{
		return nullptr;
	}
	LOGC(LInfo, "-- Finding best view to use as base among %ld views!", targetViews.size());

	std::vector<float> baseScores(targetViews.size());
#pragma omp parallel for schedule(dynamic)
	for (int i = 0; i < targetViews.size(); i++)
	{
		auto &viewPtr = targetViews[i];

		TargetTemplate3D targetTemplate = {};
		OptErrorRes baseError;
		{
			auto tgt_lock = viewPtr->target.contextualRLock();
			targetTemplate.initialise(tgt_lock->markers);

			baseError = getTargetErrorDist(calibs, *tgt_lock);
			LOGC(LDebug, "    Testing view %d for new base with a reprojection RMSE of %fpx for %d samples (%d frames and %d markers)\n",
				viewPtr->id, baseError.rmse*PixelFactor, tgt_lock->totalSamples,
				(int)tgt_lock->frames.size(), (int)tgt_lock->markers.size());
		}

		float errorSSE = 0;
		int sampleCount = 0, viewCnt = 0;
		for (auto &testViewPtr : targetViews)
		{
			if (viewPtr->id == testViewPtr->id)
				continue;
			auto tgt_lock = testViewPtr->target.contextualRLock();

			std::vector<TargetCandidate3D> alignCandidates;
			int best = findTargetAlignmentCandidates(targetTemplate, *tgt_lock, params, alignCandidates);
			if (best < 0 || std::isnan(alignCandidates[best].MSE) || alignCandidates[best].points.size() <= 4 || alignCandidates[best].MSE > 4*4)
				continue;

			LOGC(LTrace, "      Base %d can merge with target view %d with RMSE of %fmm over %d markers with %d samples total",
				viewPtr->id, testViewPtr->id, std::sqrt(alignCandidates[best].MSE), (int)alignCandidates[best].points.size(), tgt_lock->totalSamples);

			errorSSE += alignCandidates[best].MSE*tgt_lock->totalSamples;
			sampleCount += tgt_lock->totalSamples;
			viewCnt++;
		}

		float RMSE = std::sqrt(errorSSE / std::max(1, sampleCount));
		float errorScore = sampleCount / std::max(0.001f, RMSE);
		LOGC(LDebug, "    Target view %d with reprojection RMSE of %fpx had an average added RMSE of %fmm over %d markers, for a total score of %f",
			viewPtr->id, baseError.rmse*PixelFactor, std::sqrt(RMSE), sampleCount, errorScore);
		baseScores[i] = errorScore;
	}

	int bestBase = std::distance(baseScores.begin(), std::max_element(baseScores.begin(), baseScores.end()));
	LOGC(LInfo, "Picked Base Target view %d out of %ld total!", targetViews[bestBase]->id, targetViews.size());

	return targetViews[bestBase]; // new shared_ptr
}

static void ThreadCalibrationTarget(std::stop_token stopToken, PipelineState *pipeline, std::vector<CameraPipeline*> cameras)
{
	const auto exitNotifier = sg::make_scope_guard([&]() noexcept { pipeline->targetCalib.assembly.control.finished = true; });

	ScopedLogCategory scopedLogCategory(LTargetCalib, true);
	ScopedLogContext scopedLogContext(rand()); // Need something visible in the UI

	LOGC(LInfo, "=======================\n");
	LOGC(LInfo, "== Entering Target Assembly Thread:\n");

	// Copy data
	auto &assembly = pipeline->targetCalib.assembly;
	auto settings = assembly.settings;
	TargetAssemblyParameters params = pipeline->targetCalib.params.assembly;
	TargetAquisitionParameters aquisition = pipeline->targetCalib.params.aquisition;
	// Copy observation database
	std::vector<MarkerSequences> observations = pipeline->seqDatabase.contextualRLock()->markers;
	// Copy camera calibration
	std::vector<CameraCalib> calibs(cameras.size());
	for (int c = 0; c < cameras.size(); c++)
		calibs[c] = cameras[c]->calib;

	// TODO: Formulate proper plan for future of target calibration
	// Currently it is still a very manual process, so UI should be able to intervene in the algorithm
	// But it should also be redesigned to be interactive in design as initially planned

	// Idea 1: Redo target calibration as interactive algorithm
	// Base target is picked from early views
	// Additional views are merged experimentally
	// But data should be prioritised (both for merging and optimisation)
	// that overlaps with data from markers that have few data points
	// So new view, try to merge, see if new points are likely, add frames that overlap points, rest as "reserve"
	// And ofc need to speed up optimisation routine before that can become a reality

	// Idea 2: Formulate everything in a stage list
	// Start by initialising the base
	// Then follow a list of stages
	// This will first be UI only (e.g. button "Reevaluate" will add "reevaluate stage" and "optimise stage")
	// Later this will be pre-filled by automatics
	// Add view is similarly split up to allow UI to resume at last stage

	auto recordAssemblyStage = [&](TargetAssemblyBase &base, TargetAssemblyStageID stage, int step, std::string &&label) -> TargetAssemblyStage&
	{
		base.errors = getTargetErrorDist(calibs, base.target);
		base.targetTemplate = TargetTemplate3D(finaliseTargetMarkers(pipeline->getCalibs(), base.target, pipeline->targetCalib.params.post));
		auto stages_lock = pipeline->targetCalib.assemblyStages.contextualLock();

		// Log progress
		LOGC(LInfo, "Intermediary target %d - %s: Reprojection RMSE of %fpx for %d samples (%d frames and %d markers)\n",
			(int)stages_lock->size(), label.c_str(), base.errors.rmse*PixelFactor, base.target.totalSamples,
			(int)base.target.frames.size(), (int)base.target.markers.size());

		// Keep a copy of assembly stage for inspection
		stages_lock->push_back(std::make_shared<TargetAssemblyStage>(base, stage, step, std::move(label)));
		SignalPipelineUpdate();
		return *stages_lock->back();
	};

	auto performAssemblyStage = [&pipeline, &assembly, &calibs, &observations, &stopToken, &recordAssemblyStage]
		(TargetAssemblyBase &base, TargetAssemblyStageID stage, int step, TargetAssemblyParameters params, TargetAquisitionParameters aquisition)
	{
		switch (stage)
		{
			case STAGE_OPTIMISATION:
			{
				//if (getTargetErrorDist(calibs, base.optTarget).rmse > base.errors.rmse * params.optThresh)
				assembly.state.currentStage = "Optimising";
				assembly.state.optimising = true;
				assembly.state.maxSteps = assembly.state.numSteps+params.optMaxIt;
				LOGC(LInfo, "  Optimising current target:");
				ScopedLogLevel scopedLogLevel(LInfo);
				OptimisationOptions options(true, false, false, false, false);
				auto itUpdate = [&](OptErrorRes errors){
					assembly.state.errors = errors;
					assembly.state.numSteps++;
				};
				base.errors = optimiseTargetDataLoop(base.target, calibs, pipeline->frameRecords, observations, params.subsampling, aquisition,
					options, params.optMaxIt, params.optTolerance, params.outlierSigmas, stopToken, itUpdate);
				assembly.state.errors = base.errors;
				assembly.state.optimising = false;
				recordAssemblyStage(base, stage, step, "Optimised");
				break;
			}

			case STAGE_INT_ALIGN:
			{
				assembly.state.currentStage = "Aligning views";
				std::vector<TargetAlignResults> alignCandidates;
				realignTargetViewsToBase(base, *pipeline->targetCalib.views.contextualLock(), calibs, params, alignCandidates);
				recordAssemblyStage(base, stage, step, asprintf_s("Aligned views to base")).alignResults = std::move(alignCandidates);
				break;
			}

			case STAGE_INT_VIEW:
			{
				assembly.state.currentStage = "Merging views";

				// For debugging purposes
				TargetAssemblyBase preMergeBase = base;
				std::vector<TargetMergeCandidate> candidates;

				// Pick view to merge with and already merge some close markers that are likely to be one
				std::pair<int, int> viewsMerged = beginNewTargetViewMerge(base, observations, *pipeline->targetCalib.views.contextualRLock(), calibs, params, candidates);

				// For debugging purposes
				recordAssemblyStage(preMergeBase, stage, step, asprintf_s("%d views tested", viewsMerged.first)).mergeTests = std::move(candidates);

				if (!viewsMerged.first)
				{ // No merge attempts, all views are already merged
					LOGC(LInfo, "  Already merged all %d views!", (int)base.merged.size());
					base.merging = nullptr; // Just to make sure
				}
				else
				{
					recordAssemblyStage(base, stage, step, "Partially merged view");
				}
				break;
			}

			case STAGE_INT_FRAMES:
			{
				assembly.state.currentStage = "Integrating all frames";

				auto tgt = base.merging->target.contextualRLock();
				ScopedLogLevel scopedLogLevel(LInfo);
				OptimisationOptions options(true, false, false, false, false);

				// Add all remaining frames
				auto insIt = std::lower_bound(base.target.frames.begin(), base.target.frames.end(), tgt->frames.front(), 
					[&](auto &a, auto &b){ return a.frame < b.frame; });
				for (auto &frame : tgt->frames)
				{
					if (insIt != base.target.frames.end() && insIt->frame <= frame.frame)
						insIt++; // Already exists
					else
						insIt = std::next(base.target.frames.insert(insIt, frame));
				}

				// ASSERT: no duplicate frames, correct order
				long lastFrame = -1;
				for (auto &frame : base.target.frames)
				{
					assert(lastFrame < (long)frame.frame);
					lastFrame = frame.frame;
				}

				// Add more data of existing markers in new frames
				reevaluateMarkerSequences(calibs, observations, base.target, params.reevaluation);
				updateTargetObservations(base.target, observations);

				OptErrorRes newError = getTargetErrorDist(calibs, base.target);
				LOGC(LInfo, "  After adding all frames, now %d, and reevaluating marker sequences, now %d samples, left with reprojection RMSE of %fpx!",
					(int)base.target.frames.size(), base.target.totalSamples, newError.rmse*PixelFactor);

				// Only remove frames/markers below minimum data threshold, not really any outliers
				determineTargetOutliers(calibs, base.target, SigmaToErrors({ 0, 10, 10, 10 }, newError), aquisition);

				newError = getTargetErrorDist(calibs, base.target);
				LOGC(LInfo, "  After determining outliers, left with %d samples, and reprojection RMSE of %fpx!",
					base.target.totalSamples, newError.rmse*PixelFactor);

				// TODO: At this point, some frames might still be underdetermined
				// esp. if they were previously mostly determined by new markers not shared between views
				// (or not matched in reevaluateMarkerSequences)

				// Optimise frame poses and markers, but only allow removing outliers later (after reevaluateMarkerSequences)
				if (stopToken.stop_requested()) break;
				auto itUpdate = [&](OptErrorRes errors){ assembly.state.numSteps++; };
				assembly.state.optimising = true;
				assembly.state.maxSteps = assembly.state.numSteps+params.optMaxIt/2;
				optimiseTargetDataLoop(base.target, calibs, pipeline->frameRecords, observations, params.subsampling, aquisition,
					options, params.optMaxIt/2, params.optTolerance, { 1000 }, stopToken, itUpdate);
				assembly.state.optimising = false;
				if (stopToken.stop_requested()) break;
				assembly.state.optimising = true;
				assembly.state.maxSteps = assembly.state.numSteps+params.optMaxIt;
				optimiseTargetDataLoop(base.target, calibs, pipeline->frameRecords, observations, params.subsampling, aquisition,
					options, params.optMaxIt, params.optTolerance, params.outlierSigmas, stopToken, itUpdate);
				assembly.state.optimising = false;
				if (stopToken.stop_requested()) break;

				LOGC(LInfo, "  After optimising, now %d samples, optimised to reprojection RMSE of %fpx!",
					base.target.totalSamples, newError.rmse*PixelFactor);

				recordAssemblyStage(base, stage, step, "Integrated all frames");
				break;
			}

			case STAGE_INT_MARKERS:
			{
				assembly.state.currentStage = "Adopting new markers";

				auto tgt = base.merging->target.contextualRLock();
				ScopedLogLevel scopedLogLevel(LInfo);
				OptimisationOptions options(true, false, false, false, false);

				// Add remaining and new markers
				std::map<int, int> newMarkerMap;
				for (auto &m : tgt->markerMap)
				{
					auto ex = base.target.markerMap.find(m.first);
					if (ex != base.target.markerMap.end()) continue;
					auto map = newMarkerMap.find(m.second);
					if (map == newMarkerMap.end())
					{ // New marker (or one not associated with a existing one in last reevaluateMarkerSequences)
						newMarkerMap[m.second] = base.target.markers.size();
						base.target.markers.push_back(tgt->markers[m.second]);
					}
					base.target.markerMap[m.first] = newMarkerMap[m.second];
				}
				updateTargetObservations(base.target, observations);

				OptErrorRes newError = getTargetErrorDist(calibs, base.target);
				LOGC(LInfo, "  After adding remaining %d markers, now %d, left with reprojection RMSE of %fpx!",
					(int)newMarkerMap.size(), (int)base.target.markers.size(), newError.rmse*PixelFactor);

				// While adding new markers, first don't adapt frame poses at all
				//options.motion = false;

				// Optimise markers
				if (stopToken.stop_requested()) break;
				auto itUpdate = [&](OptErrorRes errors){ assembly.state.numSteps++; };
				assembly.state.optimising = true;
				assembly.state.maxSteps = assembly.state.numSteps+params.optMaxIt/2;
				optimiseTargetDataLoop(base.target, calibs, pipeline->frameRecords, observations, params.subsampling, aquisition,
					options, params.optMaxIt, params.optTolerance, params.outlierSigmas, stopToken, itUpdate);
				assembly.state.optimising = false;
				if (stopToken.stop_requested()) break;

				LOGC(LInfo, "  After adding all markers, now %d, optimised to reprojection RMSE of %fpx!",
					(int)base.target.markers.size(), newError.rmse*PixelFactor);

				// Mark view as merged into base
				base.merged.push_back(base.merging->id);
				base.alignmentStats.erase(base.merging->id);
				base.merging = nullptr;

				LOGC(LInfo, "      Ended up with merged base of %d frames, %d markers, with reprojection RMSE of %fpx!",
					(int)base.target.frames.size(), (int)base.target.markers.size(), newError.rmse*PixelFactor);

				recordAssemblyStage(base, stage, step, "Adopted new markers, view fully merged");
				break;
			}

			case STAGE_REEVALUATE_MARKERS:
			{
				assembly.state.currentStage = "Reevaluating marker sequences";

				// Add more data of existing markers in new frames, and merge markers
				determineTargetOutliers(calibs, base.target, SigmaToErrors(params.outlierSigmas, base.errors), aquisition);
				base.errors = getTargetErrorDist(calibs, base.target);
				LOGC(LInfo, "  After determining target outliers, now %d f, %d m, %d samples, left with reprojection RMSE of %fpx!",
					(int)base.target.frames.size(), (int)base.target.markers.size(), base.target.totalSamples, base.errors.rmse*PixelFactor);
				reevaluateMarkerSequences(calibs, observations, base.target, params.reevaluation);
				updateTargetObservations(base.target, observations);
				base.errors = getTargetErrorDist(calibs, base.target);
				LOGC(LInfo, "  After reevaluating marker sequences, now %d f, %d m, %d samples, left with reprojection RMSE of %fpx!",
					(int)base.target.frames.size(), (int)base.target.markers.size(), base.target.totalSamples, base.errors.rmse*PixelFactor);

				recordAssemblyStage(base, stage, step, "Reevaluated marker sequences");
				break;
			}

			case STAGE_EXPAND_FRAMES:
			{
				assembly.state.currentStage = "Expanding frames";

				expandFrameObservations(calibs, pipeline->frameRecords, base.target, base.targetTemplate, params.trackFrame);
				ReevaluateSequenceParameters reevalParams = params.reevaluation;
				reevalParams.sigmaMerge = 0; // Disable marker merging
				reevaluateMarkerSequences<true>(calibs, observations, base.target, reevalParams);
				updateTargetObservations(base.target, observations);
				base.errors = getTargetErrorDist(calibs, base.target);
				LOGC(LInfo, "  After expanding frames, now %d f, %d m, %d samples, left with reprojection RMSE of %fpx!",
					(int)base.target.frames.size(), (int)base.target.markers.size(), base.target.totalSamples, base.errors.rmse*PixelFactor);

				recordAssemblyStage(base, stage, step, "Expanded frames");
				break;
			}
			
			default:
				break;
		}
	};

	/*
	 * Initialise / Recover base
	 */
	TargetAssemblyBase base;
	int skipStep = -1;
	{
		auto stages_lock = pipeline->targetCalib.assemblyStages.contextualRLock();
		if (stages_lock->empty())
		{ // New target assembly, pick target view to use as base seed
			stages_lock.unlock();
			if (!pipeline->targetCalib.baseView)
				pipeline->targetCalib.baseView = selectCentralTargetView(*pipeline->targetCalib.views.contextualRLock(), calibs, params);
			if (!pipeline->targetCalib.baseView)
			{
				LOGC(LWarn, "-- Had no target views selected for assembly!");
				LOGC(LInfo, "=======================\n");
				SignalPipelineUpdate();
				return;
			}
			base = {};
			base.initialViewID = pipeline->targetCalib.baseView->id;
			base.merged.push_back(pipeline->targetCalib.baseView->id);
			// Copy target data
			base.target = *pipeline->targetCalib.baseView->target.contextualRLock();
			base.simulation.targetGT = pipeline->targetCalib.baseView->simulation.targetGT; // nullptr for device mode
		}
		else
		{
			base = stages_lock->back()->base;
			skipStep = stages_lock->back()->step;
		}
	}

	/*
	 * Follow manual instructions from UI
	 */
	if (!settings.followAlgorithm)
	{
		// Bit of a hack to pass custom parameters from UI to the algorithms
		params.optMaxIt = settings.maxSteps;
		params.optTolerance = settings.tolerances;

		for (TargetAssemblyStageID stage : settings.instructions)
		{
			if (stopToken.stop_requested()) break;

			performAssemblyStage(base, stage, 1, params, aquisition);

			if (stopToken.stop_requested()) break;
		}

		SignalPipelineUpdate();

		LOGC(LInfo, "=======================\n");

		return;
	}

	/*
	 * Follow algorithm
	 */
	while (true)
	{
		if (stopToken.stop_requested()) break;

		if (skipStep < 1 && base.errors.rmse > assembly.state.errors.rmse * params.optThresh)
		{
			performAssemblyStage(base, STAGE_OPTIMISATION, 1, params, aquisition);
			if (stopToken.stop_requested()) break;
		}

		// View merging step
		if (skipStep < 2)
		{
			performAssemblyStage(base, STAGE_INT_ALIGN, 2, params, aquisition);
			if (stopToken.stop_requested()) break;
		}
		if (skipStep < 3)
		{
			performAssemblyStage(base, STAGE_INT_VIEW, 3, params, aquisition);
			if (stopToken.stop_requested()) break;
		}
		if (skipStep < 4 && base.merging)
		{
			performAssemblyStage(base, STAGE_OPTIMISATION, 4, params, aquisition);
			if (stopToken.stop_requested()) break;
		}
		if (skipStep < 5 && base.merging)
		{ // Includes optimisation
			performAssemblyStage(base, STAGE_INT_FRAMES, 5, params, aquisition);
			if (stopToken.stop_requested()) break;
		}
		if (skipStep < 6 && base.merging)
		{ // Includes optimisation
			performAssemblyStage(base, STAGE_INT_MARKERS, 6, params, aquisition);
			if (stopToken.stop_requested()) break;
		}

		if (skipStep < 7 && base.errors.rmse > assembly.state.errors.rmse * params.optThresh)
		{
			performAssemblyStage(base, STAGE_OPTIMISATION, 7, params, aquisition);
			if (stopToken.stop_requested()) break;
		}

		if (skipStep < 8)
		{ // Includes optimisation
			performAssemblyStage(base, STAGE_REEVALUATE_MARKERS, 8, params, aquisition);
			if (stopToken.stop_requested()) break;
		}

		if (skipStep < 9)
		{
			performAssemblyStage(base, STAGE_OPTIMISATION, 9, params, aquisition);
			if (stopToken.stop_requested()) break;
		}
	}

	if (!stopToken.stop_requested())
	{
		performAssemblyStage(base, STAGE_OPTIMISATION, 100, params, aquisition);
	}

	if (stopToken.stop_requested())
	{
		LOGC(LInfo, "== Stopped Target Assembly!\n");
	}
	else
	{
		LOGC(LInfo, "== Finished Target Assembly!\n");
	}

	SignalPipelineUpdate();

	LOGC(LInfo, "=======================\n");
}