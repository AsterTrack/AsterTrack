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

#include "tracking/tracking.hpp"
#include "tracking/kalman.hpp"

#include "util/log.hpp"
#include "signals.hpp"

#include "flexkalman/FlexibleKalmanFilter.h"
#include "flexkalman/FlexibleUnscentedCorrect.h"

static Eigen::Vector3f AxisToUnit(TrackerAxis axis)
{
	Eigen::Vector3f unit = Eigen::Vector3f::Zero();
	unit[axis & TrackerAxis::AXIS_MASK] = (axis & TrackerAxis::AXIS_SIGN)? -1 : 1;
	return unit;
}

TrackingResult processVirtualTracker(TrackerState &state, TrackerVirtual &virt, TrackerObservation &observation,
	std::vector<TrackerState*> &subtrackers, TimePoint_t time, FrameNum frame, const VirtualTrackingParameters &params)
{
	TrackerState::Model model(params.filter.dampeningPos, params.filter.dampeningRot);

	assert(subtrackers.size() == virt.config.ids.size());
	assert(virt.config.offsetPos.size() == virt.config.offsetRot.size());

	assert(virt.config.type == VirtualTrackerType::STATIC);

	virt.relations.clear();
	virt.subtrackers.clear();
	virt.relations.reserve(subtrackers.size());
	virt.subtrackers.reserve(subtrackers.size());

	bool allTracked = !subtrackers.empty();
	int numTracked = 0;
	for (int t = 0; t < subtrackers.size(); t++)
	{
		if (subtrackers[t]) numTracked++;
		else allTracked = false;
	}
	if (numTracked == 0)
	{
		return TrackingResult::NO_TRACK;
	}
	if (frame - state.lastObsFrame > 5)
	{
		LOG(LTracking, LDebug, "Virtual Tracker detected after %" PRId64 " frames!", frame - state.lastObsFrame);
	}

	TrackingResult trackResult = TrackingResult::TRACKED_VIRTUAL;

	auto realignTracker = [&]()
	{
		// Redetermine position based on centerweights
		virt.config.centerWeights.resize(virt.config.ids.size(), 1);
		Eigen::Vector3f offset = Eigen::Vector3f::Zero();
		float weight = 0;
		for (int t = 0; t < virt.config.ids.size(); t++)
		{
			offset += virt.config.offsetPos[t] * virt.config.centerWeights[t];
			weight += virt.config.centerWeights[t];
		}
		offset /= weight;
		for (int t = 0; t < virt.config.ids.size(); t++)
			virt.config.offsetPos[t] -= offset;
		state.state.position() += offset.cast<double>();

		// Redetermine rotation
		Eigen::Quaternionf alignment = Eigen::Quaternionf::Identity();

		if (!virt.config.copyAxis.sources.empty())
		{
			Eigen::Vector3f copiedAxis = Eigen::Vector3f::Zero();
			int sources = 0;
			for (auto &source : virt.config.copyAxis.sources)
			{
				if (source.tracker < 0 || source.tracker >= virt.config.ids.size()) continue;
				sources++;
				copiedAxis += virt.config.offsetRot[source.tracker] * AxisToUnit(source.axis);
			}
			if (sources > 0)
			{
				copiedAxis /= sources;
				Eigen::Vector3f curAxis = alignment * AxisToUnit(virt.config.copyAxis.axis);
				alignment = Eigen::Quaternionf::FromTwoVectors(curAxis, copiedAxis) * alignment;
			}
		}

		if (virt.config.alignAxis.tracker >= 0 && virt.config.alignAxis.tracker < virt.config.ids.size())
		{
			Eigen::Vector3f alignAxis = virt.config.offsetPos[virt.config.alignAxis.tracker] - virt.config.centerOffset;
			Eigen::Vector3f curAxis = alignment * AxisToUnit(virt.config.alignAxis.axis);
			alignment = Eigen::Quaternionf::FromTwoVectors(curAxis, alignAxis) * alignment;
		}

		if (virt.trackerUpVector.size() == virt.config.ids.size())
		{
			Eigen::Vector3f upAxis = Eigen::Vector3f::Zero();
			for (int t = 0; t < virt.config.ids.size(); t++)
				upAxis += virt.config.offsetRot[t] * virt.trackerUpVector[t].conjugate() * Eigen::Vector3f::UnitZ();
			upAxis /= virt.config.ids.size();
			Eigen::Vector3f curAxis = alignment * AxisToUnit(virt.config.calibrateUp.axis);
			alignment = Eigen::Quaternionf::FromTwoVectors(curAxis, upAxis) * alignment;
		}

		if (virt.config.copyRotationFromTracker >= 0 && virt.config.copyRotationFromTracker < virt.config.ids.size())
		{
			alignment = virt.config.offsetRot[virt.config.copyRotationFromTracker];
		}

		// Apply new alignment
		for (int t = 0; t < virt.config.ids.size(); t++)
		{
			virt.config.offsetRot[t] = alignment.conjugate() * virt.config.offsetRot[t];
			virt.config.offsetPos[t] = alignment.conjugate() * virt.config.offsetPos[t];
		}
		state.state.setQuaternion(state.state.getCombinedQuaternion() * alignment.cast<double>());
		state.state.incrementalOrientation().setZero();

		// Apply center offset
		for (int t = 0; t < virt.config.ids.size(); t++)
			virt.config.offsetPos[t] -= virt.config.centerOffset;
		state.state.position() += virt.config.centerOffset.cast<double>();

	};

	auto calibrateTracker = [&]()
	{
		// Pick any initial center and rotation
		Eigen::Vector3f center = Eigen::Vector3f::Zero();
		for (int t = 0; t < subtrackers.size(); t++)
			center += subtrackers[t]->state.position().cast<float>();
		center /= subtrackers.size();
		Eigen::Quaternionf quat = subtrackers[0]->state.getQuaternion().cast<float>();

		// Calculate offsets based on initial transform
		virt.config.offsetPos.resize(virt.config.ids.size());
		virt.config.offsetRot.resize(virt.config.ids.size());
		for (int t = 0; t < subtrackers.size(); t++)
		{
			virt.config.offsetRot[t] = quat.conjugate() * subtrackers[t]->state.getQuaternion().cast<float>();
			virt.config.offsetPos[t] = quat.conjugate() * (subtrackers[t]->state.position().cast<float>() - center);
		}

		// Reinitialise filter
		state.state = {};
		state.state.position() = center.cast<double>();
		state.state.setQuaternion(quat.cast<double>());
		Eigen::Matrix<double,6,6> covariance = params.filter.getSyntheticCovariance<double>();
		state.state.errorCovariance().topLeftCorner<6,6>() = covariance * params.filter.sigmaInitState;
		state.state.errorCovariance().bottomRightCorner<6,6>() = covariance * params.filter.sigmaInitChange;

		// Realign based on actual config
		realignTracker();
	};

	if (state.firstObsFrame < 0 || virt.config.offsetPos.size() != virt.config.ids.size())
	{
		if (!allTracked) return TrackingResult::NO_TRACK;
		LOG(LTracking, LDebug, "Initialising virtual tracker for the first time!");
		calibrateTracker();
		virt.alignmentDirty = false;
		if (state.firstObsFrame < 0)
		{
			state.firstObsFrame = frame;
			state.firstObservation = time;
		}
	}

	if (virt.config.calibrateUp.clear)
	{
		virt.config.calibrateUp.clear = false;
		virt.collectingUpVectors = 0;
		virt.trackerUpVector.clear();
		virt.trackerUpAccum.clear();
		LOG(LTracking, LInfo, "Clearing calibration of up vector for virtual tracker!");
	}

	if (virt.alignmentDirty)
	{
		virt.alignmentDirty = false;
		realignTracker();
	}

	const int CalibrateUpNum = 20;
	if (virt.config.calibrateUp.trigger)
	{
		virt.config.calibrateUp.trigger = false;
		virt.collectingUpVectors = CalibrateUpNum;
		virt.trackerUpVector.clear();
		virt.trackerUpAccum.clear();
		virt.trackerUpAccum.resize(subtrackers.size(), Eigen::Matrix4f::Zero());
	}

	if (virt.collectingUpVectors > 0 && allTracked)
	{
		for (int t = 0; t < subtrackers.size(); t++)
		{
			Eigen::Vector4f quat = subtrackers[t]->state.getCombinedQuaternion().cast<float>().coeffs();
			if (quat.x() < 0) quat = -quat;
			virt.trackerUpAccum[t] += quat * quat.adjoint();
		}
		virt.collectingUpVectors--;
		if (virt.collectingUpVectors == 0)
		{
			virt.trackerUpVector.resize(subtrackers.size());
			bool fail = false;
			for (int t = 0; t < subtrackers.size(); t++)
			{
				Eigen::SelfAdjointEigenSolver<Eigen::Matrix4f> eigensolver(virt.trackerUpAccum[t]);
				virt.trackerUpVector[t] = Eigen::Quaternionf(eigensolver.eigenvectors().col(3)).normalized();
				if (eigensolver.info() != Eigen::Success)
				{
					LOG(LTracking, LWarn, "Failed to average rotation of tracker %d!", t);
					fail = true;
				}
			}
			virt.trackerUpAccum = {};
			if (fail)
			{
				virt.trackerUpVector.clear();
				SignalErrorToUser("Failed to calibrate up vector!");
			}
			realignTracker();
		}
	}

	// Predict new state
	flexkalman::predict(state.state, model, dtS(state.time, time));
	state.time = time;
	observation.extrapolated = state.state.getIsometry().cast<float>();
	observation.predicted = state.state.getIsometry().cast<float>();
	observation.covPredicted = state.state.errorCovariance().topLeftCorner<6,6>().cast<float>();

	// Update filter with observed tracker poses
	std::vector<Eigen::Vector3f> obsPos(subtrackers.size());
	std::vector<Eigen::Quaternionf> obsRot(subtrackers.size());
	for (int t = 0; t < subtrackers.size(); t++)
	{
		if (!subtrackers[t]) continue; // Not tracked

		// Determine virtual tracker pose from subtracker and calibrated offsets
		obsRot[t] = subtrackers[t]->state.getQuaternion().cast<float>() * virt.config.offsetRot[t].conjugate();
		Eigen::Vector3f relation = obsRot[t] * virt.config.offsetPos[t];
		obsPos[t] = subtrackers[t]->state.position().cast<float>() - relation;

		// Enter debug information for subtrackers
		Eigen::Vector3f upAxis = Eigen::Vector3f::Zero();
		if (virt.trackerUpVector.size() == subtrackers.size())
			upAxis = (subtrackers[t]->state.getQuaternion().cast<float>() * virt.trackerUpVector[t].conjugate()) * Eigen::Vector3f::UnitZ();
		virt.subtrackers.push_back({ subtrackers[t]->state.position().cast<float>(), -relation, upAxis });

		// TODO: transfer covariance from subtracker
		CovarianceMatrix virtCov = params.filter.getSyntheticCovariance<float>();

		// Use observation of subtarget to update virtual target filter
		AbsolutePoseMeasurement measurement(obsPos[t].cast<double>(), obsRot[t].cast<double>(), virtCov.cast<double>());
		flexkalman::SigmaPointParameters sigmaParams(params.filter.sigmaAlpha, params.filter.sigmaBeta, params.filter.sigmaKappa);
		bool success = flexkalman::correctUnscented(state.state, measurement, true, sigmaParams);
		if (!success)
		{ // TODO: Actually reset
			LOG(LTrackingFilter, LWarn, "Failed to correct virtual pose with pose from subtracker %d!", t);
			virt.mistrustFrames += 5;
			trackResult.setFlag(TrackingResult::FILTER_FAILED);
		}
	}

	for (int t = 0; t < subtrackers.size(); t++)
	{
		virt.relations.push_back(state.state.getQuaternion().cast<float>() * virt.config.offsetPos[t]);
	}

	if (numTracked > 1)
	{
		// Determine mistrust of current frame from tracked trackers
		float mistrust = 0.0f;
		for (int t = 0; t < subtrackers.size(); t++)
		{
			if (!subtrackers[t]) continue; // Not tracked
			float posError = (obsPos[t] - state.state.position().cast<float>()).norm() * 1000;
			float angleError = Eigen::AngleAxisf(obsRot[t] * state.state.getQuaternion().conjugate().cast<float>()).angle()/PI*180;
			LOG(LTracking, LTrace, "Virtual tracker subtracker %d had error of %.2fmm and %.2f°!", t, posError, angleError);
			mistrust += posError/20 + angleError/20;
		}

		// Accumulate frames of mistrust
		if (mistrust > subtrackers.size())
		{
			LOG(LTracking, LTrace, "Virtual tracker pose is mistrusted with rating %.4f!", mistrust);
			virt.mistrustFrames++;
		}
		else
		{
			virt.lastValidFrame = frame;
			virt.mistrustFrames--;
		}

		if (virt.mistrustFrames > 20 && allTracked)
		{ // Redetermine offsets when limit has been reached
			LOG(LTracking, LDarn, "Redetermine virtual tracker offsets after mistrust of %" PRId64 " frames (last match %" PRId64 " ago)!",
				virt.mistrustFrames, frame - virt.lastValidFrame);
			calibrateTracker();
			virt.mistrustFrames = 0;
			virt.collectingUpVectors = 0;
			virt.trackerUpAccum.clear();
			trackResult.setFlag(TrackingResult::FILTER_FAILED);
		}

		if ((params.calibration.lerpFactorPos > 0.0f || params.calibration.lerpFactorRot > 0.0f)
			&& allTracked && virt.collectingUpVectors == 0)
		{ // Slowly adapt offset to current state
			for (int t = 0; t < subtrackers.size(); t++)
			{
				if (!subtrackers[t]) continue; // Not tracked

				Eigen::Vector3f diffPos = (obsPos[t] - state.state.position().cast<float>()) * params.calibration.lerpFactorPos;
				virt.config.offsetPos[t] += obsRot[t].conjugate() * diffPos;

				Eigen::Quaternionf diffRot = obsRot[t] * state.state.getQuaternion().conjugate().cast<float>();
				Eigen::AngleAxisf diffAA(diffRot);
				diffAA.angle() *= params.calibration.lerpFactorRot;
				diffRot = diffAA;
				virt.config.offsetRot[t] = virt.config.offsetRot[t] * diffRot;
			}

			if (true)
			{ // Ensure despite lerping the desired centering/alignment is kept
				realignTracker();
			}
		}
	}

	state.lastObservation = time;
	state.lastObsFrame = frame;

	observation.filtered = state.state.getIsometry().cast<float>() ;
	observation.covFiltered = state.state.errorCovariance().topLeftCorner<6,6>().cast<float>();
	observation.time = time;

	return trackResult;
}