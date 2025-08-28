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

#include "ui/ui.hpp"

#include "pipeline/pipeline.hpp"
#include "device/tracking_camera.hpp"

#include "point/sequences2D.hpp"
#include "point/sequence_data.inl"

#include "calib/camera_system.inl"

#include "ui/gl/visualisation.hpp"

void InterfaceState::UpdateSequences(bool reset)
{
	PipelineState &pipeline = GetState().pipeline;
	if (reset) visState.incObsUpdate.reset = true;
	if (pipeline.phase == PHASE_Calibration_Point)
	{
		// Do incremental update of all observation stats and visualisations
		UpdateIncrementalSequencesVis(*pipeline.seqDatabase.contextualRLock(), true, true);

		// Not synchronised, but can read it here since it's only updated in UpdateIncrementalObservationVis before this
		auto &obsVis = visState.incObsUpdate;

		int total2DPoints = 0;
		for (int c = 0; c < pipeline.cameras.size(); c++)
			total2DPoints += obsVis.cameraTriObservations[c];
		if (obsVis.markerCount == 0)
			*calibSamples.contextualLock() = std::string("No Samples");
		else // Note: point count includes unstable, min2DPoints doesn't
			*calibSamples.contextualLock() = std::string(asprintf_s("%d samples", total2DPoints));

		return;
	}
	else if (visState.showMarkerTrails)
	{
		// Do incremental update of all observation stats and visualisations
		UpdateIncrementalSequencesVis(*pipeline.seqDatabase.contextualRLock(), false, false);

		auto frames = pipeline.record.frames.getView();
		if (!frames.empty())
		{
			auto &frame = frames.back();
			std::vector<std::vector<SceneLabel>> labels(frame->cameras.size());
			for (int c = 0; c < frame->cameras.size(); c++)
			{
				auto &cam = frame->cameras[c];
				for (int p = 0; p < cam.points2D.size(); p++)
				{
					SceneLabel label;
					label.position.head<2>() = cam.points2D[p];
					label.radius = 1 * PixelSize;
					label.text = asprintf_s("Size: %.2fpx, Value: %d", cam.properties[p].size*PixelFactor, cam.properties[p].value);
					label.color = Color{ 0.3, 0.8, 0, 1 };
					labels[c].push_back(label);
				}
			}
			for (auto &map : cameraViews)
			{
				int index = map.second.camera->pipeline->index;
				auto obs_lock = map.second.vis.observations.contextualLock();
				std::move(std::begin(labels[index]), std::end(labels[index]), std::back_inserter(obs_lock->labels));
			}

		}
	}
}

void InterfaceState::UpdateCalibrations()
{ // Check how many cameras are calibrated, and whether their calibration has been verified
	calibrationsDirty = false;
	auto &state = calibState;
	state = {};

	PipelineState &pipeline = GetState().pipeline;
	auto minTrust = pipeline.sequenceParams.get(1).FM.ConfidentMinTrust;
	auto calib_lock = pipeline.calibration.contextualRLock();
	for (int i = 0; i < pipeline.cameras.size(); i++)
	{
		if (pipeline.cameras[i]->calib.invalid())
		{
			state.numUncalibrated++;
			continue;
		}
		state.numCalibrated++;
		for (int j = 0; j < i; j++)
		{
			if (pipeline.cameras[j]->calib.invalid())
				continue;
			if (calib_lock->relations.FMStore.size() < std::min(i, j))
			{
				state.relCertain++;
				continue;
			}
			auto FMcand = calib_lock->relations.getFMEntry(i, j);
			if (FMcand.candidates.empty() || FMcand.getBestCandidate().floatingTrust < minTrust)
				state.relUncertain++;
			else
				state.relCertain++;
		}
	}

	// Tell visualisation to update precalculated calibration data
	for (auto &cam : cameraViews)
		cam.second.vis.calibration.precalculated = false;
}

void InterfaceState::UpdateIncrementalSequencesVis(const SequenceData &sequences, bool updateStable, bool rawPoints)
{
	auto &inc = visState.incObsUpdate;

	int cameraCount = sequences.temporary.size();
	int markerCount = sequences.markers.size();

	// TODO: Rework so marker merging doesn't require full re-evaluation
	// Not that easy, will need different way of keeping observation vis to update partially
	// Merging markers might add new observations in the past due to new triangulations
	// Ideally, be able to reset observations to a specific frame and update from there
	// Or only be able to update affected markers, requiring storing separate observations for each marker (yikes)
	// Probably best: Store indices every 100 frames (for the last 10000 frames) to be able to jump back to a "checkpoint"
	//if (inc.markerCount != 0 && markerCount == 0 && !inc.reset)
	if (markerCount < inc.markerCount && !inc.reset)
	{
		//assert(inc.dirty);
		LOG(LGUI, LWarn, "Incremental observation visualisation hadn't been notified of a reset in observations!");
		inc.reset = true;
	}
	if (inc.reset)
	{ // Clear state
		LOGC(LInfo, "Resetting incremental observation vis!");
		inc.reset = false;
		inc.cameraTriObservations.clear();
		inc.pointsStable = 0;
		for (auto &map : cameraViews)
		{
			map.second.vis.observations.contextualLock()->ptsStable.clear();
		}
		inc.frameStable = 0;
	}
	inc.markerCount = markerCount;

	// Init state
	inc.cameraTriObservations.resize(cameraCount, 0);

	int curStableFrame = sequences.lastRecordedFrame - stableSequenceDelay;
	int curTemporaryFrame = sequences.lastRecordedFrame - stableSequenceDelay;
	if (updateStable)
	{ // Stable Update
		std::vector<std::vector<Eigen::Vector2f>> newPointsStable(cameraCount);
		for (int m = 0; m < markerCount; m++)
		{
			const MarkerSequences &marker = sequences.markers[m];
			std::map<int, int> frameMap;
			inc.pointsStable += getTriangulationFrameMap(marker, frameMap, inc.frameStable, curStableFrame);
			handleMappedSequences(marker, frameMap, [&]
				(const PointSequence &seq, int c, int s, int seqOffset, int start, int length)
			{
				inc.cameraTriObservations[c] += length;
				auto &points = rawPoints? seq.rawPoints : seq.points;
				newPointsStable[c].insert(newPointsStable[c].end(), points.begin()+seqOffset, points.begin()+seqOffset+length);
			});
		}
		for (auto &map : cameraViews)
		{
			int index = map.second.camera->pipeline->index;
			auto obs_lock = map.second.vis.observations.contextualLock();
			for (Eigen::Vector2f &pt : newPointsStable[index])
				obs_lock->ptsStable.push_back(pt);
		}
		inc.frameStable = curStableFrame;
	}
	{ // Recreate unstable points completely (all before curStableFrame will have moved to stable)
		inc.pointsUnstable = 0;
		std::vector<std::vector<Eigen::Vector2f>> newPointsUnstable(cameraCount);
		std::vector<std::vector<Eigen::Vector2f>> newPointsTemporary(cameraCount);
		std::vector<std::vector<Eigen::Vector2f>> newPointsInactive(cameraCount);
		for (int m = 0; m < markerCount; m++)
		{
			const MarkerSequences &marker = sequences.markers[m];
			if (marker.lastFrame < curStableFrame) continue;

			/* std::map<int, int> frameMap;
			inc.pointsUnstable += getTriangulationFrameMap(marker, frameMap, curStableFrame, curTemporaryFrame);
			handleMappedSequences(marker, frameMap, [&]
				(const PointSequence &seq, int c, int s, int seqOffset, int start, int length)
			{
				// inc.cameraTriObservations[c] is incremental - just don't include unstable, it's fine
				auto &points = rawPoints? seq.rawPoints : seq.points;
				newPointsUnstable[c].insert(newPointsUnstable[c].end(), points.begin()+seqOffset, points.begin()+seqOffset+length);
			}); */

			for (int c = 0; c < marker.cameras.size(); c++)
			{
				const CameraSequences &camSeq = marker.cameras[c];
				auto frame = camSeq.upper(curTemporaryFrame);
				while (frame != camSeq.end())
				{
					if (camSeq.sequences[frame.seq].isInactive())
						newPointsInactive[c].push_back(rawPoints? frame.raw() : *frame);
					else
					 	newPointsUnstable[c].push_back(rawPoints? frame.raw() : *frame);
					frame++;
				}
			}
		}
		for (int c = 0; c < cameraCount; c++)
		{
			for (int t = 0; t < sequences.temporary[c].size(); t++)
			{
				auto &temp = sequences.temporary[c][t];
				auto &points = rawPoints? temp.rawPoints : temp.points;
				if (temp.isInactive())
					newPointsInactive[c].insert(newPointsInactive[c].end(), points.begin(), points.end());
				else
					newPointsTemporary[c].insert(newPointsTemporary[c].end(), points.begin(), points.end());
			}
		}
		for (auto &map : cameraViews)
		{
			int index = map.second.camera->pipeline->index;
			auto obs_lock = map.second.vis.observations.contextualLock();
			obs_lock->ptsUnstable = std::move(newPointsUnstable[index]);
			obs_lock->ptsTemp = std::move(newPointsTemporary[index]);
			obs_lock->ptsInactive = std::move(newPointsInactive[index]);
		}
	}
	{
		std::vector<std::vector<SceneLabel>> labels(sequences.temporary.size());
		for (int m = 0; m < markerCount; m++)
		{
			const MarkerSequences &marker = sequences.markers[m];
			if (marker.lastFrame < sequences.lastRecordedFrame-5) continue;

			for (int c = 0; c < marker.cameras.size(); c++)
			{
				const CameraSequences &camSeq = marker.cameras[c];
				if (camSeq.sequences.empty() || camSeq.sequences.back().lastFrame() < sequences.lastRecordedFrame-5)
					continue;
				auto &seq = camSeq.sequences.back();

				SceneLabel label;
				label.position.head<2>() = (rawPoints? seq.rawPoints.back() : seq.points.back()) + Eigen::Vector2f(0.0f, 1*PixelSize);
				label.radius = 1 * PixelSize;
				label.text = asprintf_s("Value: %.2f", seq.value);
				label.color = Color{ 0, 0.4f, 0.8f, 1 };
				labels[c].push_back(label);
			}
		}
		for (int c = 0; c < cameraCount; c++)
		{
			for (int t = 0; t < sequences.temporary[c].size(); t++)
			{
				auto &seq = sequences.temporary[c][t];
				SceneLabel label;
				label.position.head<2>() = (rawPoints? seq.rawPoints.back() : seq.points.back()) + Eigen::Vector2f(0.0f, 1*PixelSize);
				label.radius = 1 * PixelSize;
				label.text = asprintf_s("Value: %.2f", seq.value);
				label.color = Color{ 0.8f, 0.3f, 0.0f, 1 };
				labels[c].push_back(label);
			}
		}
		for (auto &map : cameraViews)
		{
			int index = map.second.camera->pipeline->index;
			auto obs_lock = map.second.vis.observations.contextualLock();
			obs_lock->labels = std::move(labels[index]);
		}
	}
}

CameraConfig& getCameraConfig(const TrackingCameraState &camera)
{
	return GetState().cameraConfig.getCameraConfig(camera.id);
}

std::vector<std::string> getAbnormalStatus(const TrackingCameraState &camera, bool &abnormalStreamingState)
{
	auto status = *camera.state.contextualRLock(); // Copy
	std::vector<std::string> statusMsgs;
	if (GetState().mode != MODE_Device)
	{
		abnormalStreamingState = false;
		return statusMsgs;
	}

	// Detect if camera may need a streaming restart
	bool expectingStreaming = GetState().isStreaming && !status.error.encountered;
	bool isCameraStreaming = camera.isStreaming();// && camera.state.fsEnabled;
	bool recentlySendCommand = camera.hasSetStreaming() && dtMS(camera.modeSet.time, sclock::now()) < 1000;
	abnormalStreamingState = expectingStreaming && !isCameraStreaming && !recentlySendCommand;

	if ((status.commState & CommReady) != CommReady)
	{
		// Currently this implies (status.hadMCUConnected || status.hadPiConnected)
		if ((status.commState & CommReady) != CommNoCon || dtMS(status.lastConnecting, sclock::now()) < 1000)
			statusMsgs.push_back("Camera is trying to reconnect...");
		else
			statusMsgs.push_back("Camera is disconnected");
	}
	else if (status.commState == CommMCUReady)
	{
		statusMsgs.push_back("Camera is connected and starting up...");
	}

	// Display recent error
	if (status.error.encountered)
		statusMsgs.push_back(asprintf_s("'%s' (Recovering)", ErrorTag_String[status.error.code]));
	else if (status.error.recovered && dtMS(status.error.recoverTime, sclock::now()) < 3000)
		statusMsgs.push_back(asprintf_s("'%s' (Recovered)", ErrorTag_String[status.error.code]));
	else if (expectingStreaming && !isCameraStreaming)
	{ // May happen if controller stopped streaming on it's own and told cameras to stop, too
		// e.g. when server hung for several seconds in debug
		statusMsgs.push_back("Camera is not streaming!");
	}

	return statusMsgs;
}

std::string getStatusText(const TrackingCameraState &camera)
{
	auto status = *camera.state.contextualRLock(); // Copy
	if (GetState().mode != MODE_Device)
	{
		return "Camera.";
	}

	// Detect if camera may need a streaming restart
	bool expectingStreaming = GetState().mode == MODE_Device && GetState().isStreaming && !status.error.encountered;
	bool isCameraStreaming = camera.isStreaming();// && camera.state.fsEnabled;
	//bool recentlySendCommand = camera.hasSetStreaming() && dtMS(camera.modeSet.time, sclock::now()) < 1000;
	bool abnormalStreamingState = expectingStreaming && !isCameraStreaming;

	if (status.error.encountered)
	{
		if ((status.commState & CommReady) != CommReady)
		{
			// Currently this implies (status.hadMCUConnected || status.hadPiConnected)
			if ((status.commState & CommReady) != CommNoCon || dtMS(status.lastConnecting, sclock::now()) < 1000)
				return asprintf_s("Camera encountered error '%s' and is currently trying to reconnect.", ErrorTag_String[status.error.code]);
			else
				return asprintf_s("Camera encountered error '%s' and is disconnected.", ErrorTag_String[status.error.code]);
		}
		else
		{
			assert(status.commState == CommMCUReady);
			return asprintf_s("Camera encountered error '%s' and is currently recovering.", ErrorTag_String[status.error.code]);
		}
	}
	else if ((status.commState & CommReady) != CommReady)
	{
		// Currently this implies (status.hadMCUConnected || status.hadPiConnected)
		if ((status.commState & CommReady) != CommNoCon || dtMS(status.lastConnecting, sclock::now()) < 1000)
			return "Camera is trying to reconnect...";
		else
			return "Camera is disconnected";
	}
	else if (status.commState == CommMCUReady)
	{
		return "Camera is connected and starting up...";
	}
	else
	{
		assert(status.commState == CommPiReady);
		if (status.error.recovered && abnormalStreamingState)
		{
			return asprintf_s("Camera recovered from error '%s' but is not yet streaming!", ErrorTag_String[status.error.code]);
		}
		else if (status.error.recovered && dtMS(status.error.recoverTime, sclock::now()) < 3000)
		{
			return asprintf_s("Camera recovered from error '%s'", ErrorTag_String[status.error.code]);
		}
		else if (abnormalStreamingState)
		{ // May happen if controller stopped streaming on it's own and told cameras to stop, too
			// e.g. when server hung for several seconds in debug
			return "Camera is not streaming!";
		}
		else if (status.error.code != ERROR_NONE)
		{
			return asprintf_s("Camera is working properly.\n Last Error was '%s'.", ErrorTag_String[status.error.code]);
		}
		else
		{
			return "Camera is working properly.";
		}
	}
}

Color getStatusColor(const TrackingCameraState &camera)
{
	ServerState &state = GetState();
	bool streamingMatch = (state.mode != MODE_Device) || (camera.isStreaming() == state.isStreaming);
	Color statusColor = { 0, 1, 0, 1.0f };
	auto camState = *camera.state.contextualRLock(); // Copy
	auto &error = camState.error;
	if (error.encountered)
		statusColor = { 1, 0, 0, 1.0f };
	else if (!streamingMatch)
		statusColor = { 1, 1, 0, 1.0f };
	statusColor.a *= 0.5f;
	return statusColor;
}

Color getStatusColor(const TrackingControllerState &controller)
{
	Color statusColor = { 0, 1, 0, 1.0f };
	statusColor.a *= 0.5f;
	// TODO: Map status of controller to a color
	return statusColor;
}