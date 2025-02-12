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

#include "calib_target/assembly.hpp"
#include "ui.hpp"
#include "ui/system/vis.hpp"

#include "pipeline/pipeline.hpp" // TargetView
#include "calib/obs_data.hpp"

#include "device/tracking_controller.hpp"
#include "comm/usb.hpp"

#include "gl/visualisation.hpp" // Color
#include "imguizmo/ImSequencer.hpp"
#include "implot/implot.h"


struct TargetViewSequence : public ImSequencer::SequenceInterface
{
private:
	VisualisationState *m_vis;
	int state;
	intptr_t tgtPtr;

	struct MarkerSequence
	{
		int index;
		int frameStart, frameEnd;
		std::vector<unsigned int> colorBand;
		std::vector<float> rawErrors;
		std::vector<int> observations;
	};
	std::vector<MarkerSequence> m_markers;

public:
	int m_start;
	int frameCount, markerCount, sampleCount;

	bool needsUpdate(const VisTargetLock &visTarget)
	{
		return tgtPtr != (intptr_t)visTarget.target || sampleCount != visTarget.target->totalSamples
			|| frameCount != visTarget.target->frames.size() || markerCount != visTarget.target->markers.size();
		//	|| state != (m_view->calibrated? m_view->calibration->contextualRLock()->numSteps : -1);
	}

	TargetViewSequence(const VisTargetLock &visTarget, VisualisationState &visState, const std::vector<CameraCalib> &calibs)
		: m_vis(&visState), m_start(0)
	{
		assert(visTarget.target);
		//if (m_view)
		//	state = m_view->calibrated? m_view->calibration->contextualRLock()->numSteps : -1;
		tgtPtr = (intptr_t)visTarget.target;
		sampleCount = visTarget.target->totalSamples;
		frameCount = visTarget.target->frames.size();
		markerCount = visTarget.target->markers.size();
		m_markers.resize(visTarget.target->markers.size());
		for (int i = 0; i < m_markers.size(); i++)
		{
			m_markers[i].index = i;
			m_markers[i].frameStart = -1;
			m_markers[i].frameEnd = -1;
			m_markers[i].colorBand.clear();
			m_markers[i].colorBand.reserve(frameCount);
			m_markers[i].rawErrors.clear();
			m_markers[i].rawErrors.reserve(frameCount);
			m_markers[i].observations.clear();
			m_markers[i].observations.reserve(frameCount);
		}
		int f = 0;
		for (auto &frame : visTarget.target->frames)
		{
			for (auto &sample : frame.samples)
			{
				int m = visTarget.target->markerMap.at(sample.marker);
				auto &marker = m_markers[m];
				if (marker.frameStart < 0 || marker.frameStart > f)
					marker.frameStart = f;
				if (marker.frameEnd < f)
					marker.frameEnd = f;
				// Evaluate observation error
				Eigen::Vector3f markerPt = visTarget.target->markers[m];
				Eigen::Vector2f proj = projectPoint2D(calibs[sample.camera].camera, frame.pose * markerPt);
				Eigen::Vector2f point = undistortPoint<float>(calibs[sample.camera], sample.point);
				float error = (point - proj).norm();
				// Update marker observation sequence
				int i = f - marker.frameStart;
				if (marker.observations.size() <= i)
				{
					marker.observations.resize(i+1, 0);
					marker.rawErrors.resize(i+1, -1.0f);
					marker.colorBand.resize(i+1, 0);
				}
				marker.rawErrors[i] = (marker.rawErrors[i]*marker.observations[i] + error) / (marker.observations[i]+1);
				marker.observations[i]++;
				// Determine new color
				const float errorHigh = 2*PixelSize, errorExtreme = 10*PixelSize;
				Color mix;
				if (marker.rawErrors[i] < errorHigh)
					mix = lerp({ 0, 1, 0, 1 }, { 1, 0, 0, 1 }, marker.rawErrors[i]/errorHigh);
				else
				 	mix = lerp({ 1, 0, 0, 1 }, { 1, 0, 1, 1 }, marker.rawErrors[i]/errorExtreme);
				marker.colorBand[i] = IM_COL32(mix.r*255, mix.g*255, mix.b*255, mix.a*255);
			}
			f++;
		}
		
		drawSequenceBars = false;
	}

	virtual int GetFrameMin() const { return 0; }
	virtual int GetFrameMax() const { return frameCount-1; }
	virtual int GetItemCount() const { return markerCount; }
	virtual int GetItemTypeCount() const { return 1; }
	virtual const char* GetItemTypeName(int typeIndex) const { return ""; }
	virtual const char* GetItemLabel(int index) const
	{
		static char tmps[512];
		const MarkerSequence &marker = m_markers[index];
		int frame = GetUI().visState.targetCalib.frameIdx;
		float curSampleError = -1.0f;
		if (marker.frameStart <= frame && marker.frameEnd >= frame)
			curSampleError = marker.rawErrors[frame-marker.frameStart];
		if (curSampleError >= 0.0f) // Marker visible in current frame
			snprintf(tmps, sizeof(tmps), "Marker %d (%.2fpx)", index, curSampleError*PixelFactor);
		else
			snprintf(tmps, sizeof(tmps), "Marker %d", index);
		return tmps;
	}

	virtual void Get(int index, int** start, int** end, int* type, unsigned int* color)
	{
		MarkerSequence& marker = m_markers[index];
		if (color)
			*color = m_vis->targetCalib.markerSelect[index]? IM_COL32(0xFF, 0xDD, 0x00, 0xFF) : IM_COL32(0x80, 0x80, 0xDD, 0xFF);
		if (start)
			*start = &marker.frameStart;
		if (end)
			*end = &marker.frameEnd;
		if (type)
			*type = 0;
	}

	virtual void DoubleClick(int index)
	{
		m_vis->targetCalib.markerSelect[index] = !m_vis->targetCalib.markerSelect[index];
	}

	virtual void CustomDrawCompact(int index, ImDrawList* draw_list, const ImRect& rc, const ImRect& clippingRect)
	{
		MarkerSequence &marker = m_markers[index];
		if (marker.colorBand.size() < 2) return;
		float dx = (rc.Max.x - rc.Min.x)/frameCount;
		draw_list->PushClipRect(clippingRect.Min, clippingRect.Max, true);
		if (m_vis->targetCalib.markerSelect[index])
		{ // Draw marked background
			draw_list->AddRectFilled(clippingRect.Min, clippingRect.Max, IM_COL32(200, 200, 200, 160));
		}
		{ // Draw begin cap
			ImVec2 min(rc.Min.x + dx * (marker.frameStart - 1.5f), rc.Min.y + 5);
			ImVec2 max(min.x + dx, rc.Max.y - 3);
			draw_list->AddRectFilledMultiColor(min, max,
				IM_COL32(0,0,0,0), marker.colorBand.front(),
				marker.colorBand.front(), IM_COL32(0,0,0,0));
		}
		{ // Draw end cap
			ImVec2 min(rc.Min.x + dx * (marker.frameStart + marker.colorBand.size() - 1.5f), rc.Min.y + 5);
			ImVec2 max(min.x + dx, rc.Max.y - 3);
			draw_list->AddRectFilledMultiColor(min, max,
				marker.colorBand.back(), IM_COL32(0,0,0,0),
				IM_COL32(0,0,0,0), marker.colorBand.back());
		}
		for (int i = 0; i < marker.colorBand.size()-1; i++)
		{
			ImVec2 min(rc.Min.x + dx * (marker.frameStart + i - 0.5f), rc.Min.y + 5);
			ImVec2 max(min.x + dx, rc.Max.y - 3);
			draw_list->AddRectFilledMultiColor(min, max,
				marker.colorBand[i], marker.colorBand[i+1],
				marker.colorBand[i+1], marker.colorBand[i]);
			if (marker.observations[i] > 1)
			{
				Color mix = lerp({ 1, 1, 0, 1 }, { 0, 1, 1, 1 }, (marker.observations[i]-1)/3);
				ImVec2 mid(min.x, rc.Max.y - 5);
				draw_list->AddRectFilled(mid, max, IM_COL32(mix.r*255, mix.g*255, mix.b*255, mix.a*255));
			}
		}
		draw_list->PopClipRect();
	}
};

struct TrackingControllerEventSequence : public ImSequencer::SequenceInterface
{
private:
	long offsetUS = 0, lengthUS = 1000;
	long frontIndex = -1, backIndex = -1;

	// Cached rendering structure
	std::array<std::vector<std::pair<float,float>>, CONTROLLER_EVENT_MAX> evtBars;

	// Visible events
	std::array<bool, CONTROLLER_EVENT_MAX> evtFilter;
	std::vector<ControllerEventID> evtList;

	// Cached data for determining if end frame was changed by user (invalidating followLastFrame)
	int intendedPos = 0;
	float intendedWidth;

public:
	std::shared_ptr<TrackingControllerState> m_controller;

	int viewStartTimeUS = 0;
	bool followLastFrame = true;

	TrackingControllerEventSequence(std::shared_ptr<TrackingControllerState> controller)
		: m_controller(std::move(controller))
	{
		auto events = m_controller->eventLog.getView();
		if (!events.empty())
			offsetUS = (long)events.front().timestampUS-10;
	}

	void UpdateEventList(std::vector<ControllerEventID> eventList)
	{
		evtList = std::move(eventList);
		for (int i = 0; i < CONTROLLER_EVENT_MAX; i++)
			evtFilter[i] = false;
		for (ControllerEventID evt : evtList)
		{
			assert(evt < CONTROLLER_EVENT_MAX);
			evtFilter[evt] = true;
		}
		frontIndex = backIndex = -1; // Force evtBars to be rebuilt
	}

	virtual int GetFrameMin() const { return offsetUS; }
	virtual int GetFrameMax() const { return offsetUS+lengthUS; }
	virtual int GetItemCount() const { return evtList.size(); }
	virtual int GetItemTypeCount() const { return 1; }
	virtual const char* GetItemTypeName(int typeIndex) const { return ""; }
	virtual const char* GetItemLabel(int i) const
	{
		return getControllerEventName((ControllerEventID)evtList[i]);
	}

	void SetViewIntervalUS(float intervalUS)
	{
		framePixelWidthTarget = framePixelWidth = intendedWidth/intervalUS;
		intendedPos = 0;
	}

	virtual void PrepareRendering(float sequenceWidth)
	{
		auto events = m_controller->eventLog.getView();
		if (events.empty()) return;

		if (offsetUS == 0)
		{ // Don't update when culling, keep original timeline range
			offsetUS = (long)events.front().timestampUS-10;
			if (viewStartTimeUS < offsetUS)
				viewStartTimeUS = offsetUS;
		}
		lengthUS = std::max((long)1000, (long)events.back().timestampUS+10-offsetUS);

		// Before this, viewStartTimeUS (as firstFrame) could've been modified by panning (should disable followLastFrame)
		// And visibleFrameCount can change, so have to take start pos
		if (intendedPos && followLastFrame && intendedPos < viewStartTimeUS)
		{ // Start pos set in PostRenderUpdate
			followLastFrame = false;
			intendedPos = 0;
		}
		// Modifying firstFrame of sequence is supported here - visibleFrameCount will be updated
		if (followLastFrame)
		{
			viewStartTimeUS = GetFrameMax()-visibleFrameCount;
			// After this, viewStartTimeUS (as firstFrame) could be modified by scrollbar (only end pos should disable followLastFrame)
			// However, visibleFrameCount will not be updated (smoothed over time), so have to recalculate with width/framePixelWidthTarget
			intendedPos = viewStartTimeUS+visibleFrameCount; // Set end pos for PostRenderUpdate
		}
		intendedWidth = sequenceWidth;

		// This whole update only takes about 0.5ms even with about 40000 events

		long frontUS = viewStartTimeUS, backUS = frontUS+visibleFrameCount;
		auto frontIt = std::lower_bound(events.begin(), events.end(), frontUS,
		[](const ControllerEventLog& event, long us) { return event.timestampUS < us; });
		auto backIt = std::upper_bound(events.begin(), events.end(), backUS,
		[](int us, const ControllerEventLog& event) { return us < event.timestampUS; });
		if (backIt != events.begin())
			backIt--;
		if (frontIt.index() == frontIndex && backIt.index() == backIndex)
			return; // No update needed
		frontIndex = frontIt.index();
		backIndex = backIt.index();

		LOG(LGUI, LTrace, "Rendering events from index %ld (%fus) to %ld (%f us), viewing %ldus - %ldus (start %d, width %d, offset %ld), first has timestamp %ld (%fus)",
			frontIndex, frontIt->timestampUS, backIndex, backIt->timestampUS, frontUS, backUS, viewStartTimeUS, visibleFrameCount, offsetUS, events.front().timestamp, events.front().timestampUS);

		float dUS = sequenceWidth/visibleFrameCount;
		float mergeLimit = 1.5f/dUS; // 1px from minimum width of rendering, the rest to avoid flickering

		std::array<std::pair<float,float>, CONTROLLER_EVENT_MAX> render;
		for (int i = 0; i < CONTROLLER_EVENT_MAX; i++)
		{
			float invisStart = (float)(viewStartTimeUS - GetFrameMin()) - 2*dUS;
			render[i] = { invisStart, invisStart };
			evtBars[i].clear();
		}
		for (auto p = frontIt; p <= backIt; p++)
		{
			if (!evtFilter[p->id]) continue;
			auto &evtRender = render[p->id];
			double pos = p->timestampUS - GetFrameMin();
			if (!p->isNewEvent)
			{ // Have to add to current bar because it's ending
				if (evtRender.second < pos)
					evtRender.second = pos;
			}
			else if (pos-evtRender.second < mergeLimit)
			{ // Will merge with bar because it's too close to differentiate anyway
				evtRender.second = pos;
			}
			else
			{ // Last bar is done, start new
				evtBars[p->id].push_back(evtRender);
				evtRender.first = pos;
				evtRender.second = pos;
			}
		}
		for (int i = 0; i < CONTROLLER_EVENT_MAX; i++)
		{
			auto &evtRender = render[i];
			// TODO: Minor glitch since there's no data to differentiate singular event and beginning of sequence
			/* if (lastIsBegin)
				evtRender.second = viewStartTimeUS+visibleFrameCount; */
			if (evtRender.first != evtRender.second)
			{ // Finish last bar
				evtBars[i].push_back(evtRender);
			}
		}
	}

	void PostRenderUpdate()
	{
		// framePixelWidthTarget might've just been updated via scrollbar zooming
		// Need to check if scrollbar editing caused end frame to change
		int newVisibleFrameCount = intendedWidth/framePixelWidthTarget;
		if (followLastFrame && intendedPos < viewStartTimeUS+newVisibleFrameCount)
		{ // End pos set in PrepareRendering
			followLastFrame = false;
			intendedPos = 0;
		}
		
		if (followLastFrame)
		{ // Set start pos for PrepareRendering
			intendedPos = viewStartTimeUS;
		}
	}

	virtual void Get(int index, int** start, int** end, int* type, unsigned int* color)
	{
		static int z = 0;
		if (color)
			*color = IM_COL32(0x00, 0x00, 0x00, 0x00);
		if (start)
			*start = &z;
		if (end)
			*end = &z;
		if (type)
			*type = 0;
	}

	virtual void CustomDrawCompact(int i, ImDrawList* draw_list, const ImRect& rc, const ImRect& clippingRect)
	{
		if (evtList.size() <= i) return;
		draw_list->PushClipRect(clippingRect.Min, clippingRect.Max, true);
		float width = rc.Max.x - rc.Min.x;
		float dUS = width/(GetFrameMax()-GetFrameMin()+1);
		auto &bars = evtBars[evtList[i]];
		for (auto bar : bars)
		{
			ImVec2 min(rc.Min.x + dUS * bar.first - 0.5f, rc.Min.y + 5);
			ImVec2 max(rc.Min.x + dUS * bar.second + 0.5f, rc.Max.y - 3);
			draw_list->AddRectFilled(min, max, IM_COL32(0xFF, 0x00, 0x00, 0xFF));
		}
		draw_list->PopClipRect();
	}
};

template<> void OpaqueDeleter<TargetViewSequence>::operator()(TargetViewSequence* ptr) const
{ delete ptr; }
template<> void OpaqueDeleter<TrackingControllerEventSequence>::operator()(TrackingControllerEventSequence* ptr) const
{ delete ptr; }

static bool ShowTrackingPanel();
static void CleanTrackingPanel();
static bool ShowTargetCalibrationPanel(VisTargetLock &visTarget);
static void CleanTargetCalibrationPanel();
static bool ShowTrackingControllerPanel();
static void CleanTrackingControllerPanel();
static bool ShowTimingPanel();
static void CleanTimingPanel();

void InterfaceState::UpdateInsights(InterfaceWindow &window)
{
	static int lastPanel = 0;
	auto clean = []()
	{
		if (lastPanel == 1)
			CleanTargetCalibrationPanel();
		else if (lastPanel == 2)
			CleanTrackingControllerPanel();
		else if (lastPanel == 3)
			CleanTimingPanel();
		lastPanel = 0;
	};
	if (!ImGui::Begin(window.title.c_str(), &window.open))
	{
		clean();
		ImGui::End();
		return;
	}
	if (!ImGui::BeginTabBar("InsightsSelector", ImGuiTabBarFlags_None))
	{
		clean();
		ImGui::End();
		return;
	}

	ServerState &state = GetState();
	PipelineState &pipeline = state.pipeline;

	int curPanel = -1;

	if (pipeline.phase == PHASE_Tracking && ImGui::BeginTabItem("Tracking"))
	{
		if (!ShowTrackingPanel())
			CleanTrackingPanel();
		else
			curPanel = 1;
		ImGui::EndTabItem();
	}
	else if (lastPanel == 1)
		CleanTrackingPanel();

	{
		VisTargetLock visTarget = visState.lockVisTarget();
		if ((visTarget || pipeline.phase == PHASE_Calibration_Target)
			&& ImGui::BeginTabItem("Target Calibration"))
		{
			if (!ShowTargetCalibrationPanel(visTarget))
				CleanTargetCalibrationPanel();
			else
				curPanel = 2;
			ImGui::EndTabItem();
		}
		else if (lastPanel == 2)
			CleanTargetCalibrationPanel();
	}
	
	if (state.mode == MODE_Device && !state.controllers.empty() && ImGui::BeginTabItem("Tracking Controller"))
	{
		if (!ShowTrackingControllerPanel())
			CleanTrackingControllerPanel();
		else
			curPanel = 3;
		ImGui::EndTabItem();
	}
	else if (lastPanel == 3)
		CleanTrackingControllerPanel();

	if (state.mode == MODE_Device && !state.controllers.empty() && ImGui::BeginTabItem("Timing"))
	{
		if (!ShowTimingPanel())
			CleanTimingPanel();
		else
			curPanel = 4;
		ImGui::EndTabItem();
	}
	else if (lastPanel == 4)
		CleanTimingPanel();

	lastPanel = curPanel;

	ImGui::EndTabBar();
	ImGui::End();
}

static bool ShowTrackingPanel()
{
	InterfaceState &ui = GetUI();
	ServerState &state = GetState();
	PipelineState &pipeline = state.pipeline;

	static int curTargetID = 0;
	static std::string curTargetLabel = "Select Target";
	if (ImGui::BeginCombo("Target", curTargetLabel.c_str(), ImGuiComboFlags_WidthFitPreview))
	{
		bool changed = false;
		auto TargetSelect = [&changed](const TargetTemplate3D &target)
		{
			ImGui::PushID(target.id);
			bool selected = curTargetID == target.id;
			if (ImGui::Selectable(target.label.c_str(), selected) && curTargetID != target.id)
			{
				curTargetID = target.id;
				curTargetLabel = target.label;
				changed = true;
			}
			ImGui::PopID();
		};
		for (auto &target : pipeline.tracking.trackedTargets)
			TargetSelect(*target.target);
		for (auto &target : pipeline.tracking.lostTargets)
			TargetSelect(*target.first);
		ImGui::EndCombo();
		if (changed)
			ImGui::MarkItemEdited(ImGui::GetItemID());
	}
	if (curTargetID == 0 && (!pipeline.tracking.trackedTargets.empty() || !pipeline.tracking.lostTargets.empty()))
	{
		auto target = !pipeline.tracking.trackedTargets.empty()? pipeline.tracking.trackedTargets.front().target : pipeline.tracking.lostTargets.front().first;
		curTargetID = target->id;
		curTargetLabel = target->label;
	}

	static bool followFrame = true, showCur = true, showRec = true;
	ImGui::SameLine();
	ImGui::Checkbox("Follow Frame", &followFrame);
	ImGui::SameLine();
	ImGui::Checkbox("Current", &showCur);
	if (state.mode == MODE_Replay)
	{
		ImGui::SameLine();
		ImGui::Checkbox("Recorded", &showRec);
	}

	auto frames = pipeline.frameRecords.getView();
	unsigned int frameNum = pipeline.frameNum.load();
	if (curTargetID != 0 && (!frames.empty() || !state.loadedFrameRecords.empty())
		&& ImPlot::BeginPlot("##RealtimeTracking", ImVec2(-1, -1)))
	{
		static struct {
			bool isTracking;
			std::vector<int> sampleCount;
			std::vector<float> errors2D;
			std::vector<float> tracking;
			std::vector<float> targetLost, targetAttempts, targetDetected, targetCatchup;

			void setup(int size)
			{
				isTracking = false;
				sampleCount.clear();
				errors2D.clear();
				tracking.clear();
				sampleCount.resize(size, 0);
				errors2D.resize(size, NAN);
				tracking.resize(size, NAN);
				targetLost.clear();
				targetDetected.clear();
				targetAttempts.clear();
				targetCatchup.clear();
			}
		} tracking, recording;

		auto updateFrameStats = [&](auto &stats, unsigned int index, FrameRecord &frame, float evtOffset = 0.0f)
		{
			auto tracked = std::find_if(frame.tracking.targets.begin(), frame.tracking.targets.end(), [&](auto &t){ return t.id == curTargetID; });
			{ // Events
				int maxEvents = frame.tracking.trackingLosses + frame.tracking.searches2D + frame.tracking.detections3D + frame.tracking.trackingCatchups;
				float step = std::min(0.4f, 1.0f/maxEvents), pos = -step * (maxEvents-1)/2 - step + evtOffset*step;
				for (int i = 0; i < frame.tracking.trackingLosses; i++)
					stats.targetLost.push_back(frame.num + (pos += step));
				for (int i = 0; i < frame.tracking.searches2D-frame.tracking.detections2D; i++)
					stats.targetAttempts.push_back(frame.num + (pos += step));
				for (int i = 0; i < frame.tracking.detections2D; i++)
					stats.targetDetected.push_back(frame.num + (pos += step));
				for (int i = 0; i < frame.tracking.detections3D; i++)
					stats.targetDetected.push_back(frame.num + (pos += step));
				for (int i = 0; i < frame.tracking.trackingCatchups; i++)
					stats.targetCatchup.push_back(frame.num + (pos += step));
			}
			if (tracked == frame.tracking.targets.end())
			{
				stats.isTracking = false;
				return;
			}
			if (!stats.isTracking)
				stats.isTracking = true;
			stats.tracking[index] = true;
			stats.sampleCount[index] = tracked->sampleCnt;
			stats.errors2D[index] = tracked->error2DAvg * PixelFactor;
		};

		// Update frameRange
    	static ImPlotRange frameRange(0, 1000);
		int maxOffset = std::max(10.0, frameRange.Size()/6);
		if (followFrame)
		{ // Apply offset due to new recent frame
			double offset = 0;
			if (frameRange.Max < frameNum+maxOffset)
				offset = std::max(frameNum + maxOffset - frameRange.Max, -frameRange.Min);
			else if (frameRange.Min > frameNum-maxOffset)
				offset = std::max(frameNum - maxOffset - frameRange.Min, -frameRange.Min);
			frameRange.Min += offset;
			frameRange.Max += offset;
		}
		double framesAxisMax = std::max((double)std::max(frames.endIndex(), state.loadedFrameRecords.size()), frameRange.Max);

		// Setup plots
		ImPlot::SetupAxis(ImAxis_X1, "Frames",ImPlotAxisFlags_NoLabel);
		ImPlot::SetupAxisLimits(ImAxis_X1, 0, framesAxisMax);
		ImPlot::SetupAxis(ImAxis_Y1, "Samples",ImPlotAxisFlags_Lock);
		ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 50);
		ImPlot::SetupAxis(ImAxis_Y2, "Errors /px", ImPlotAxisFlags_Opposite | ImPlotAxisFlags_Lock);
		ImPlot::SetupAxisLimits(ImAxis_Y2, 0, 1.0f);
		ImPlot::SetupAxisLinks(ImAxis_X1, &frameRange.Min, &frameRange.Max);

		// Update frameRange from inputs
		ImPlot::SetupFinish(); // Will process inputs, but not update frameRange yet (EndPlot does)
		frameRange = ImPlot::GetPlotLimits(ImAxis_X1).X;
		// TODO: Bit of jelly in tracking graph since there's no true constraints to ensure frameRange.Max == framesAxisMax after input

		double frameShift, frameShiftRec;
		bool drawCur = false, drawRec = false;

		if (showCur && !frames.empty())
		{ // Gather stats for realtime tracking data
			long long min = std::max<long long>(frameRange.Min-1, frames.beginIndex());
			long long max = std::min<long long>(frameRange.Max+1, frames.endIndex()-1);
			if (max >= min)
			{
				std::size_t frameCnt = max-min+1;
				tracking.setup(frameCnt);
				auto frameIt = frames.pos(max);
				for (int i = 0; i < frameCnt; i++, frameIt--)
				{
					if (!*frameIt || !frameIt->get()->finishedProcessing) continue;
					updateFrameStats(tracking, frameCnt-i-1, *frameIt->get(), 0.0f);
				}
				frameShift = (double)min;
				drawCur = frameCnt > 0;
			}
			GetUI().RequestUpdates();
		}

		if (showRec && state.mode == MODE_Replay && !state.loadedFrameRecords.empty())
		{ // Gather stats for recorded tracking data
			long long min = std::max<long long>(frameRange.Min-1, 0);
			long long max = std::min<long long>(frameRange.Max+1, state.loadedFrameRecords.size()-1);
			if (max >= min)
			{
				std::size_t frameCnt = max-min+1;
				recording.setup(frameCnt);
				auto frameIt = state.loadedFrameRecords.begin() + max;
				for (int i = 0; i < frameCnt; i++, frameIt--)
				{
					updateFrameStats(recording, frameCnt-i-1, *frameIt, 0.4f);
				}
				frameShiftRec = (double)min;
				drawRec = frameCnt > 0;
			}
		}

		// Sample count bars
		ImPlot::SetAxis(ImAxis_Y1);
		if (drawCur)
		{ // Draw current
			ImPlot::SetNextLineStyle(ImVec4(0.3*1.2, 0.45*1.2, 0.7*1.2, 1.0));
			ImPlot::SetNextFillStyle(ImVec4(0.3*1.2, 0.45*1.2, 0.7*1.2, 1.0));
			ImPlot::PlotBars("Samples", tracking.sampleCount.data(), tracking.sampleCount.size(), 0.67f, frameShift);
		}
		if (drawRec)
		{ // Draw recorded
			ImPlot::SetNextLineStyle(ImVec4(0.3*0.8, 0.45*0.8, 0.7*0.8, 0.6));
			ImPlot::SetNextFillStyle(ImVec4(0.3*0.8, 0.45*0.8, 0.7*0.8, 0.6));
			ImPlot::PlotBars("Samples Recorded", recording.sampleCount.data(), recording.sampleCount.size(), 0.67f, frameShiftRec);
		}

		// Error line
		ImPlot::SetAxis(ImAxis_Y2);
		if (drawRec)
		{ // Draw recorded
			ImPlot::SetNextLineStyle(ImVec4(0.87*0.6, 0.52*0.6, 0.32*0.6, 1.0), 2.0);
			ImPlot::PlotLine("Errors Recorded", recording.errors2D.data(), recording.errors2D.size(), 1, frameShiftRec);
		}
		if (drawCur)
		{ // Draw current
			ImPlot::SetNextLineStyle(ImVec4(0.87, 0.52, 0.32, 1), 2.0);
			ImPlot::PlotLine("Errors", tracking.errors2D.data(), tracking.errors2D.size(), 1, frameShift);
		}

		// Events
		if (drawRec)
		{ // Draw recorded
			ImPlot::SetNextLineStyle(ImVec4(1, 0, 0, 0.7), 1.0);
			ImPlot::PlotInfLines("##LossRec", recording.targetLost.data(), recording.targetLost.size());
			ImPlot::SetNextLineStyle(ImVec4(0, 1, 0, 0.7), 1.0);
			ImPlot::PlotInfLines("##DetectRec", recording.targetDetected.data(), recording.targetDetected.size());
			ImPlot::SetNextLineStyle(ImVec4(0.8, 0.8, 0, 0.7), 1.0);
			ImPlot::PlotInfLines("##AttemptsRec", recording.targetAttempts.data(), recording.targetAttempts.size());
			ImPlot::SetNextLineStyle(ImVec4(0.8, 0.8, 0.2, 1), 1.0);
			ImPlot::PlotInfLines("##CatchupRec", recording.targetCatchup.data(), recording.targetCatchup.size());
		}
		if (drawCur)
		{ // Draw current
			ImPlot::SetNextLineStyle(ImVec4(1, 0, 0, 1), 1.0);
			ImPlot::PlotInfLines("##Loss", tracking.targetLost.data(), tracking.targetLost.size());
			ImPlot::SetNextLineStyle(ImVec4(0, 1, 0, 1), 1.0);
			ImPlot::PlotInfLines("##Detect", tracking.targetDetected.data(), tracking.targetDetected.size());
			ImPlot::SetNextLineStyle(ImVec4(0.8, 0.8, 0, 1), 1.0);
			ImPlot::PlotInfLines("##Attempts", tracking.targetAttempts.data(), tracking.targetAttempts.size());
			ImPlot::SetNextLineStyle(ImVec4(0.8, 0.8, 0.2, 1), 1.0);
			ImPlot::PlotInfLines("##Catchup", tracking.targetCatchup.data(), tracking.targetCatchup.size());
		}

		// Tracking state
		if (drawRec)
		{ // Draw recorded
			ImPlot::SetNextFillStyle(ImVec4(0.33*0.6, 0.66*0.6, 0.4*0.6, 0.8));
			ImPlot::PlotDigital("##TrackingRec", recording.tracking.data(), recording.tracking.size(), frameShiftRec);
		}
		if (drawCur)
		{ // Draw current
			ImPlot::SetNextFillStyle(ImVec4(0.33, 0.66, 0.4, 1.0));
			ImPlot::PlotDigital("##Tracking", tracking.tracking.data(), tracking.tracking.size(), frameShift);
		}

		// Current and selected frames
		double curFrame = frameNum + 0.5;
		double jumpFrame = (double)ui.frameJumpTarget;
		ImPlot::SetNextLineStyle(ImVec4(0.33, 0.66, 0.4, 1.0), 3);
		ImPlot::PlotInfLines("CurFrame", &curFrame, 1);
		if (ImPlot::DragLineX(2, &jumpFrame, ImVec4(0.66, 0.33, 0.4, 1.0), 3))
			ui.frameJumpTarget = (unsigned int)jumpFrame;

		ImPlot::EndPlot();
	}

	return true;
}
static void CleanTrackingPanel()
{
	InterfaceState &ui = GetUI();
}

const static char* lastTargetTab = nullptr;
static bool ShowTargetCalibrationPanel(VisTargetLock &visTarget)
{
	if (!ImGui::BeginTabBar("TargetCalibSelector", ImGuiTabBarFlags_None))
		return false;

	InterfaceState &ui = GetUI();
	ServerState &state = GetState();
	PipelineState &pipeline = state.pipeline;

	const char* targetAquisitionLabel = "Aquisition";
	const char* targetMarkerLabel = "Target Markers";
	const char* curTab = nullptr;

	if (pipeline.phase == PHASE_Calibration_Target && ImGui::BeginTabItem(targetAquisitionLabel))
	{
		curTab = targetAquisitionLabel;

		if (ImPlot::BeginPlot("##Aquisition", ImVec2(-1, -1)))
		{
			auto aquisition_lock = pipeline.targetCalib.aquisition.contextualRLock();
			unsigned int frameNum = pipeline.frameNum.load();
			ImPlot::PushColormap(ImPlotColormap_Deep);

            ImPlot::SetupAxis(ImAxis_X1, "Frames",ImPlotAxisFlags_Lock | ImPlotAxisFlags_NoLabel);
            ImPlot::SetupAxisLimits(ImAxis_X1, 0, TargetViewAquisitionWindowSize);

            ImPlot::SetupAxis(ImAxis_Y1, "Markers",ImPlotAxisFlags_LockMin);
            ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 20);

			//ImPlot::SetNextLineStyle(ImVec4(1, 1, 1, 1));
			ImPlot::PlotLine("Avg Marker Count", 
				aquisition_lock->avgMarkerCount.data(), aquisition_lock->avgMarkerCount.size(), 
				1, 0, ImPlotLineFlags_None,
				&aquisition_lock->avgMarkerCount.front()-aquisition_lock->avgMarkerCount.data());

			//ImPlot::SetNextLineStyle(ImVec4(0, 0, 1, 0.8f));
			ImPlot::PlotLine("Shared Marker Count", 
				aquisition_lock->sharedMarkerCount.data(), aquisition_lock->sharedMarkerCount.size(), 
				1, 0, ImPlotLineFlags_None,
				&aquisition_lock->sharedMarkerCount.front()-aquisition_lock->sharedMarkerCount.data());

			double limit = pipeline.targetCalib.params.aquisition.minSharedMarkerCount;
			if (ImPlot::DragLineY(1, &limit, ImPlot::GetLastItemColor()))
				pipeline.targetCalib.params.aquisition.minSharedMarkerCount = (int)limit;
	
			//ImPlot::SetNextLineStyle(ImVec4(0, 1, 0, 0.8f));
			ImPlot::PlotLine("Value", 
				aquisition_lock->valueGraph.data(), aquisition_lock->valueGraph.size(), 
				1, 0, ImPlotLineFlags_None,
				&aquisition_lock->valueGraph.front()-aquisition_lock->valueGraph.data());

			if (aquisition_lock->localMaxSearching)
			{
				double value[] = { (double)(aquisition_lock->localMaxFrame - frameNum + TargetViewAquisitionWindowSize) };
				ImPlot::PlotInfLines("##Searching", value, 1);
			}

			std::vector<double> searchBounds;
			std::vector<double> searchMaxima;
			for (auto it = aquisition_lock->localMaximas.rbegin(); it != aquisition_lock->localMaximas.rend(); ++it)
			{
				if (it->end < frameNum-TargetViewAquisitionWindowSize) break;
				searchBounds.push_back((double)(it->begin - frameNum + TargetViewAquisitionWindowSize));
				searchBounds.push_back((double)(it->end - frameNum + TargetViewAquisitionWindowSize));
				searchMaxima.push_back((double)(it->maxima - frameNum + TargetViewAquisitionWindowSize));
			}
			ImPlot::PlotInfLines("##Bounds", searchBounds.data(), searchBounds.size());
			ImPlot::PlotInfLines("##Maxima", searchMaxima.data(), searchMaxima.size());

			ImPlot::PopColormap();
			ImPlot::EndPlot();

			if (pipeline.recordSequences)
				ui.RequestUpdates();
		}

		ImGui::EndTabItem();
	}

	static float targetViewZoom = 10.0f;

	if (visTarget && ImGui::BeginTabItem(targetMarkerLabel))
	{
		curTab = targetMarkerLabel;

		ui.visState.updateVisTarget(visTarget);

		if (!ui.seqTarget || ui.seqTarget->needsUpdate(visTarget))
		{ // Need to update/recreate sequence
			std::vector<CameraCalib> calibs;
			for (const auto &cam : pipeline.cameras)
				calibs.push_back(cam->calib);
			ui.seqTarget = make_opaque<TargetViewSequence>(visTarget, ui.visState, calibs);

			// Keep zoom intact
			ui.seqTarget->framePixelWidthTarget = targetViewZoom;
			ui.seqTarget->framePixelWidth = targetViewZoom;
		}
		targetViewZoom = ui.seqTarget->framePixelWidthTarget;

		ImSequencer::Sequencer(ui.seqTarget.get(), &ui.visState.targetCalib.frameIdx, nullptr, &ui.visState.targetCalib.markerFocussed, &ui.seqTarget->m_start,
			ImSequencer::SEQUENCER_CHANGE_FRAME);

		ImGui::EndTabItem();
	}

	if ((curTab == nullptr || curTab == targetAquisitionLabel) && lastTargetTab != nullptr)
	{
		CleanTargetCalibrationPanel();
	}

	lastTargetTab = curTab;
	ImGui::EndTabBar();
	return true;
}
static void CleanTargetCalibrationPanel()
{
	InterfaceState &ui = GetUI();
	if (ui.seqTarget)
	{
		ui.seqTarget = nullptr;
	}
	lastTargetTab = nullptr;
}

const static char* lastControllerTab = nullptr;
static bool ShowTrackingControllerPanel()
{
	if (!ImGui::BeginTabBar("ControllerSelector", ImGuiTabBarFlags_None))
		return false;

	InterfaceState &ui = GetUI();
	ServerState &state = GetState();
	PipelineState &pipeline = state.pipeline;

	// TODO: Allow exact selection of events, and send them to controller, not as a class of events
	static std::vector<ControllerEventID> eventSelection;
	static int eventClassCode = 0;

	// TODO: Allow selection of controller for event viewer
	auto &controller = state.controllers.front();

	if (controller->eventLog.getView().empty())
	{
		ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(LogLevelHexColors[LWarn]), "Please make sure the controller firmware has been built with Event Logging support! Release builds have not.");
	}

	/* Choose events to view */

	const char* curTab = nullptr;
	if (ImGui::BeginTabItem("Interrupts"))
	{
		curTab = "Interrupts";
		eventClassCode = 1;
		eventSelection = {
			CONTROLLER_INTERRUPT_USB,
			CONTROLLER_INTERRUPT_UART,
			CONTROLLER_INTERRUPT_SYNC_GEN,
			CONTROLLER_INTERRUPT_SYNC_INPUT,
			CONTROLLER_INTERRUPT_LED_UPDATE,
			CONTROLLER_INTERRUPT_PD_INT,
			CONTROLLER_INTERRUPT_FLASH_BUTTON,
			CONTROLLER_INTERRUPT_FLASH_TIMER
		};
		ImGui::EndTabItem();
	}
	if (ImGui::BeginTabItem("USB Events"))
	{
		curTab = "USB Events";
		eventClassCode = 2;
		eventSelection = {
			CONTROLLER_EVENT_USB_SOF,
			CONTROLLER_EVENT_USB_CONTROL,
			CONTROLLER_EVENT_USB_DATA_TX
		};
		ImGui::EndTabItem();
	}
	if (ImGui::BeginTabItem("Streaming"))
	{
		curTab = "Streaming";
		eventClassCode = 3;
		eventSelection = {
			CONTROLLER_EVENT_SYNC,
			CONTROLLER_EVENT_DATA_IN,
			CONTROLLER_EVENT_DATA_OUT
		};
		ImGui::EndTabItem();
	}
	if (ImGui::BeginTabItem("Sending"))
	{
		curTab = "Sending";
		eventClassCode = 4;
		eventSelection = {
			CONTROLLER_EVENT_USB_SENDING_NULL,
			CONTROLLER_EVENT_USB_SENDING_PACKET,
			CONTROLLER_EVENT_USB_QUEUE_SOF
		};
		ImGui::EndTabItem();
	}

	if (!curTab)
	{
		ImGui::EndTabBar();
		return false;
	}

	/* Update event sequence with new tab */

	if (ui.seqEvents && ui.seqEvents->m_controller != controller)
	{
		comm_submit_control_data(ui.seqEvents->m_controller->comm, COMMAND_OUT_EVENTS, 0x00, 0, nullptr, 0);
		ui.seqEvents = nullptr;
	}

	if (lastControllerTab != curTab || !ui.seqEvents)
	{
		if (!ui.seqEvents)
		{
			ui.seqJumpToFrame = -1;
			ui.seqEvents = make_opaque<TrackingControllerEventSequence>(controller);
		}
		ui.seqEvents->UpdateEventList(eventSelection);
		if (ui.seqEventsActive)
			comm_submit_control_data(controller->comm, COMMAND_OUT_EVENTS, 0x00, eventClassCode, nullptr, 0);
		else
			comm_submit_control_data(controller->comm, COMMAND_OUT_EVENTS, 0x00, 0, nullptr, 0);
	}
	lastControllerTab = curTab;

	/* Show controls for sequence */

	if (ImGui::TabItemButton("Follow frame", ImGuiTabItemFlags_Trailing | ImGuiTabItemFlags_NoTooltip))
		ui.seqEvents->followLastFrame = true;

	ImGui::BeginDisabled(!controller->sync);
	if (ImGui::TabItemButton("Set Frame Interval", ImGuiTabItemFlags_Trailing | ImGuiTabItemFlags_NoTooltip))
		ui.seqEvents->SetViewIntervalUS(controller->sync->contextualRLock()->frameIntervalMS*1000.0f);
	ImGui::EndDisabled();

	ImGui::BeginDisabled(!controller->comm->commStreaming);
	if (ImGui::TabItemButton(ui.seqEventsActive? "Disable##Toggle" : "Enable##Toggle", ImGuiTabItemFlags_Trailing | ImGuiTabItemFlags_NoTooltip))
	{
		ui.seqEventsActive = !ui.seqEventsActive;
		if (ui.seqEventsActive)
			comm_submit_control_data(controller->comm, COMMAND_OUT_EVENTS, 0x00, eventClassCode, nullptr, 0);
		else
			comm_submit_control_data(controller->comm, COMMAND_OUT_EVENTS, 0x00, 0, nullptr, 0);
	}
	ImGui::EndDisabled();

	/* Show sequencer */

	static long markedFrame;
	if (markedFrame != ui.seqJumpToFrame)
	{
		markedFrame = ui.seqJumpToFrame;
		if (ui.seqJumpToFrame >= 0)
		{
			ui.seqEvents->viewStartTimeUS = ui.seqJumpToFrame - ui.seqEvents->visibleFrameCount/2;
			ui.seqEvents->followLastFrame = false;
		}
	}

	// Draw sequencer viewer
	int curFrame = (int)ui.seqJumpToFrame;
	ImSequencer::Sequencer(ui.seqEvents.get(), &curFrame, nullptr, nullptr, &ui.seqEvents->viewStartTimeUS,
		ImSequencer::SEQUENCER_EDIT_NONE);
	if (ui.seqEvents->GetItemCount() > 0)
	{ // Else rendering, and thus PrepareRendering, is skipped
		ui.seqEvents->PostRenderUpdate();
	}

	ImGui::EndTabBar();
	return true;
}
static void CleanTrackingControllerPanel()
{
	InterfaceState &ui = GetUI();
	if (ui.seqEvents)
	{
		comm_submit_control_data(ui.seqEvents->m_controller->comm, COMMAND_OUT_EVENTS, 0x00, 0, nullptr, 0);
		ui.seqEvents = nullptr;
	}
	ui.seqJumpToFrame = -1;
	ui.seqEventsActive = false;
	lastControllerTab = nullptr;
}
static bool ShowTimingPanel()
{
	InterfaceState &ui = GetUI();

	auto &controller = *GetState().controllers.front();
	controller.recordTimeSyncMeasurements = true;
	controller.timeSyncMeasurements.delete_culled();
	auto samples = controller.timeSyncMeasurements.getView();

	static std::size_t graphBegin = 0, graphEnd = 0;
	bool updateGraph = graphBegin != samples.beginIndex() || graphEnd != samples.endIndex();
	if (updateGraph)
	{
		graphBegin = samples.beginIndex();
		graphEnd = samples.endIndex();
	}

	// Reference point to hold on to
	static int initConfidence = 0;
	static TimePoint_t referenceTime;
	static long long referenceTimestamp;
	if (initConfidence < 50 && !samples.empty())
	{
		int index = std::max(samples.beginIndex(), std::min(samples.endIndex()-1, (std::size_t)50));
		auto &sample = samples[index];
		referenceTimestamp = sample.timestamp;
		referenceTime = sample.measurement;
		initConfidence = index;
	}

	if (controller.comm->commStreaming)
		ui.RequestUpdates();

	static TimeSync init = {};
	static bool emulate = false;
	updateGraph |= ImGui::Checkbox("Enable Emulation", &emulate);
	if (emulate)
	{
		updateGraph |= ImGui::SliderFloat("Drift Lerp", &init.driftLerp, 0, 0.000001f, "%.10ff", ImGuiSliderFlags_NoRoundToFormat);
		updateGraph |= ImGui::SliderFloat("Drift Bias", &init.driftBias, -0.001f, 0.001f, "%.10ff", ImGuiSliderFlags_NoRoundToFormat);
		updateGraph |= ImGui::InputInt("Drift Init Range", &init.driftInitRange, 0, 10000);
		updateGraph |= ImGui::SliderFloat("Drift Init Adapt", &init.driftInitAdapt, 0.0f, 1000.0f);
		updateGraph |= ImGui::SliderFloat("Drift Downward Correct", &init.driftDownwardCorrect, 0.0f, 1000.0f);
		updateGraph |= ImGui::SliderFloat("Drift Downward Jump", &init.driftDownwardJump, 0.0f, 1.0f);
		updateGraph |= ImGui::InputInt("Offset US", &init.timeOffsetUS);
	}

	if (ImPlot::BeginPlot("##TimeSync", ImVec2(-1, -1)))
	{

		/*
			While the controller time itself is not drift-free, it is the only certain, unchanging measurement
			As the mapped real time changes with the time sync parameters
			So the controller time (in US) is the basis and x-axis of the visualisation

			For every packet, we record its controller time (in us) and its receive time on the host 
			The Y axis shall then represent both the drift correction of timesync, and the latency of the individual packet
				this is achieved by correcting all Y values from the diagonal to the horizontal X axis
				e.g. subtracting the time that SHOULD have passed if there was no drift
			As such, the bottom end of the record will be at the time we THINK is was sent,
				and the top when it was received, representing the latency
		*/

		ImPlot::SetupAxis(ImAxis_X1, "Controller Time (us)");

		auto formatter = [](double value, char* buff, int size, void* data)
		{
			double width = ImPlot::GetPlotLimits(ImAxis_X1, ImAxis_Y1).Size().x;
			long timeUS = (long)value;
			if (width > 10000000)
				return snprintf(buff, size, "%lds", timeUS/1000000);
			else if (width > 500000)
				return snprintf(buff, size, "%ldms", timeUS%10000000/1000);
			else if (width > 10000)
				return snprintf(buff, size, "%ldms", timeUS%1000000/1000);
			else if (width > 500)
				return snprintf(buff, size, "%ldus", timeUS%10000);
			else
				return snprintf(buff, size, "%ldus", timeUS%1000);
		};

		ImPlot::SetupAxisFormat(ImAxis_X1, formatter);
		ImPlot::SetupAxis(ImAxis_Y1, "Drift / Latency (us)");

		thread_local std::vector<double> controllerTimes;
		thread_local std::vector<double> measuredTimesOffset;
		thread_local std::vector<double> estimatedTimesOffset;
		thread_local std::vector<double> emulatedTimesOffset;
		thread_local std::vector<double> resetEventX;
		thread_local std::vector<double> resetEventY;

		if (updateGraph)
		{
			controllerTimes.clear();
			measuredTimesOffset.clear();
			estimatedTimesOffset.clear();
			emulatedTimesOffset.clear();
			resetEventX.clear();
			resetEventY.clear();

			controllerTimes.reserve(samples.size());
			measuredTimesOffset.reserve(samples.size());
			estimatedTimesOffset.reserve(samples.size());
			emulatedTimesOffset.reserve(samples.size());

			TimeSync emulatedTimeSync = init;

			for (const auto &sample : samples)
			{
				long long baseUS = sample.timestamp - referenceTimestamp;
				controllerTimes.push_back((double)baseUS);
				measuredTimesOffset.push_back((double)(dtUS(referenceTime, sample.measurement) - baseUS));
				estimatedTimesOffset.push_back((double)(dtUS(referenceTime, sample.estimation) - baseUS));

				if (emulate)
				{
					TimePoint_t emulated = UpdateTimeSync(emulatedTimeSync, sample.timestamp&(((uint64_t)1<<63) -1), (uint64_t)1<<63, sample.measurement).second;
					emulatedTimesOffset.push_back((double)(dtUS(referenceTime, emulated) - baseUS));
					if (emulatedTimeSync.measurements == 1)
					{
						resetEventX.push_back((double)baseUS);
						resetEventY.push_back((double)(dtUS(referenceTime, sample.measurement) - baseUS));
					}
				}
			}
		}

		ImPlot::PushStyleVar(ImPlotStyleVar_MarkerSize, 2);
		ImPlot::PlotScatter("Measurements", controllerTimes.data(), measuredTimesOffset.data(), controllerTimes.size(), ImPlotLineFlags_None);
		ImPlot::PlotScatter("Estimations", controllerTimes.data(), estimatedTimesOffset.data(), controllerTimes.size(), ImPlotLineFlags_None);
		if (emulate)
		{
			ImPlot::PlotScatter("Emulated Estimations", controllerTimes.data(), emulatedTimesOffset.data(), controllerTimes.size(), ImPlotLineFlags_None);
			ImPlot::PlotScatter("Emulated Resets", resetEventX.data(), resetEventY.data(), resetEventX.size(), ImPlotLineFlags_None);
		}
		ImPlot::PopStyleVar();

		ImPlot::EndPlot();
	}

	// Blocks of 4096 measurements each - keep only last 10
	controller.timeSyncMeasurements.cull_front(-10);

	return true;
}
static void CleanTimingPanel()
{
	for (auto &controller : GetState().controllers)
	{
		controller->recordTimeSyncMeasurements = false;
		controller->timeSyncMeasurements.cull_clear();
	}

	for (auto &controller : GetState().controllers)
	{
		controller->timeSyncMeasurements.delete_culled();
	}
}