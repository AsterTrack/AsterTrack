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

#include "ui.hpp"
#include "ui/system/vis.hpp"

#include "pipeline/pipeline.hpp" // TargetView
#include "calib/obs_data.hpp"

#include "point/sequence_data.inl"

#include "device/tracking_controller.hpp"
#include "comm/usb.hpp"

#include "gl/visualisation.hpp" // Color
#include "imguizmo/ImSequencer.hpp"
#include "implot/implot.h"

#include "ctpl/ctpl.hpp"
#include <filesystem>
extern ctpl::thread_pool threadPool;

#include "nativefiledialog-extended/nfd.h"
#include "nativefiledialog-extended/nfd_glfw3.h"

#include <fstream>


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
		return tgtPtr != (intptr_t)visTarget.obs || sampleCount != visTarget.obs->totalSamples
			|| frameCount != visTarget.obs->frames.size() || markerCount != visTarget.obs->markers.size();
		//	|| state != (m_view->calibrated? m_view->calibration->contextualRLock()->numSteps : -1);
	}

	TargetViewSequence(const VisTargetLock &visTarget, VisualisationState &visState, const std::vector<CameraCalib> &calibs)
		: m_vis(&visState), m_start(0)
	{
		assert(visTarget.obs);
		//if (m_view)
		//	state = m_view->calibrated? m_view->calibration->contextualRLock()->numSteps : -1;
		tgtPtr = (intptr_t)visTarget.obs;
		sampleCount = visTarget.obs->totalSamples;
		frameCount = visTarget.obs->frames.size();
		markerCount = visTarget.obs->markers.size();
		m_markers.resize(visTarget.obs->markers.size());
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
		for (auto &frame : visTarget.obs->frames)
		{
			for (auto &sample : frame.samples)
			{
				int m = visTarget.obs->markerMap.at(sample.marker);
				auto &marker = m_markers[m];
				if (marker.frameStart < 0 || marker.frameStart > f)
					marker.frameStart = f;
				if (marker.frameEnd < f)
					marker.frameEnd = f;
				// Evaluate observation error
				Eigen::Vector3f markerPt = visTarget.obs->markers[m];
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
			*color = m_vis->target.markerSelect[index]? IM_COL32(0xFF, 0xDD, 0x00, 0xFF) : IM_COL32(0x80, 0x80, 0xDD, 0xFF);
		if (start)
			*start = &marker.frameStart;
		if (end)
			*end = &marker.frameEnd;
		if (type)
			*type = 0;
	}

	virtual void DoubleClick(int index)
	{
		m_vis->target.markerSelect[index] = !m_vis->target.markerSelect[index];
	}

	virtual void CustomDrawCompact(int index, ImDrawList* draw_list, const ImRect& rc, const ImRect& clippingRect)
	{
		MarkerSequence &marker = m_markers[index];
		if (marker.colorBand.size() < 2) return;
		float dx = (rc.Max.x - rc.Min.x)/frameCount;
		draw_list->PushClipRect(clippingRect.Min, clippingRect.Max, true);
		if (m_vis->target.markerSelect[index])
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
				Color mix = lerp({ 1, 1, 0, 1 }, { 0, 1, 1, 1 }, (marker.observations[i]-1)/3.0f);
				ImVec2 mid(min.x, rc.Max.y - 5);
				draw_list->AddRectFilled(mid, max, IM_COL32(mix.r*255, mix.g*255, mix.b*255, mix.a*255));
			}
		}
		draw_list->PopClipRect();
	}
};


struct Marker2DSequence : public ImSequencer::SequenceInterface
{
private:
	VisualisationState *m_vis;
	int state;

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
	int lastFrame, markerCount;
	std::pair<int,int> totalFrameRange;

	bool needsUpdate(const SequenceData &sequence)
	{
		return lastFrame != sequence.lastRecordedFrame || markerCount != sequence.markers.size();
	}

	Marker2DSequence(const SequenceData &sequence, VisualisationState &visState, const CameraSystemCalibration &calib)
		: m_vis(&visState)
	{
		//if (m_view)
		//	state = m_view->calibrated? m_view->calibration->contextualRLock()->numSteps : -1;
		lastFrame = sequence.lastRecordedFrame;
		markerCount = sequence.markers.size();
		int camCount = sequence.temporary.size();
		//m_markers.reserve(sequence.markers.size() * camCount);
		m_markers.resize(sequence.markers.size());

		totalFrameRange = { lastFrame, lastFrame };
		for (int i = 0; i < sequence.markers.size(); i++)
		{
			auto frameRange = sequence.markers[i].getFrameRange();
			totalFrameRange.first = std::min(totalFrameRange.first, frameRange.first);
		}
		m_start = totalFrameRange.first;

		for (int i = 0; i < sequence.markers.size(); i++)
		{
			auto frameRange = sequence.markers[i].getFrameRange();
			int frameCount = frameRange.second - frameRange.first;
			//std::vector<MarkerSequences> markerSeq(camCount);
			auto &marker = m_markers[i];
			marker.index = i;
			marker.frameStart = frameRange.first;
			marker.frameEnd = frameRange.second;
			marker.colorBand.clear();
			marker.colorBand.resize(frameCount, IM_COL32(0, 0, 0, 0));
			marker.rawErrors.clear();
			marker.rawErrors.resize(frameCount);
			marker.observations.clear();
			marker.observations.resize(frameCount);

			std::map<int, int> frameMap;
			getObservationFrameMap(sequence.markers[i], frameMap, frameRange.first, frameRange.second);
			handleMappedSequences(sequence.markers[i], frameMap, [&]
				(const PointSequence &seq, int c, int s, int seqOffset, int start, int length)
			{
				for (int p = 0; p < seq.length(); p++)
				{
					int index = seq.startFrame + p - marker.frameStart;
					marker.observations[index]++;
					marker.colorBand[index] = IM_COL32(0.1*255, 0.3*255, 0.4*255, 0.8*255);
				}
			});

			getTriangulationFrameMap(sequence.markers[i], frameMap, frameRange.first, frameRange.second);
			handleMappedSequences(sequence.markers[i], frameMap, [&]
				(const PointSequence &seq, int c, int s, int seqOffset, int start, int length)
			{
				for (int p = 0; p < seq.length(); p++)
				{
					int index = seq.startFrame + p - marker.frameStart;
					// Determine new color
					Color mix = lerp({ 0.1, 0.3, 0.4, 0.8}, { 0.1, 1.0, 0.2, 1.0 }, (float)(marker.observations[index]-1) / camCount);
					marker.colorBand[index] = IM_COL32(mix.r*255, mix.g*255, mix.b*255, mix.a*255);
				}
			});

			marker.frameStart -= m_start;
			marker.frameEnd -= m_start;
		}

		drawSequenceBars = false;
	}

	virtual int GetFrameMin() const { return totalFrameRange.first; }
	virtual int GetFrameMax() const { return totalFrameRange.second-1; }
	virtual int GetItemCount() const { return markerCount; }
	virtual int GetItemTypeCount() const { return 1; }
	virtual const char* GetItemTypeName(int typeIndex) const { return ""; }
	virtual const char* GetItemLabel(int index) const
	{
		static char tmps[512];
		const MarkerSequence &marker = m_markers[index];
		/* int frame = GetUI().visState.targetCalib.frameIdx;
		float curSampleError = -1.0f;
		if (marker.frameStart <= frame && marker.frameEnd >= frame)
			curSampleError = marker.rawErrors[frame-marker.frameStart];
		if (curSampleError >= 0.0f) // Marker visible in current frame
			snprintf(tmps, sizeof(tmps), "Marker %d (%.2fpx)", index, curSampleError*PixelFactor);
		else */
		snprintf(tmps, sizeof(tmps), "Marker %d", index);
		return tmps;
	}

	virtual void Get(int index, int** start, int** end, int* type, unsigned int* color)
	{
		MarkerSequence& marker = m_markers[index];
		if (color)
			*color = IM_COL32(0x80, 0x80, 0xDD, 0xFF);
		if (start)
			*start = &marker.frameStart;
		if (end)
			*end = &marker.frameEnd;
		if (type)
			*type = 0;
	}

	virtual void DoubleClick(int index)
	{
		m_vis->target.markerSelect[index] = !m_vis->target.markerSelect[index];
	}

	virtual void CustomDrawCompact(int index, ImDrawList* draw_list, const ImRect& rc, const ImRect& clippingRect)
	{
		MarkerSequence &marker = m_markers[index];
		if (marker.colorBand.size() < 2) return;
		float dx = (rc.Max.x - rc.Min.x)/(totalFrameRange.second-totalFrameRange.first);
		draw_list->PushClipRect(clippingRect.Min, clippingRect.Max, true);
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
				Color mix = lerp({ 1, 1, 0, 1 }, { 0, 1, 1, 1 }, (marker.observations[i]-1)/3.0f);
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
template<> void OpaqueDeleter<Marker2DSequence>::operator()(Marker2DSequence* ptr) const
{ delete ptr; }
template<> void OpaqueDeleter<TrackingControllerEventSequence>::operator()(TrackingControllerEventSequence* ptr) const
{ delete ptr; }

static bool ShowTrackingPanel();
static void CleanTrackingPanel();
static bool ShowTargetCalibrationPanel(VisTargetLock &visTarget);
static void CleanTargetCalibrationPanel();
static bool ShowSequencePanel();
static void CleanSequencePanel();
static bool ShowTrackingControllerPanel();
static void CleanTrackingControllerPanel();
static bool ShowTimingPanel();
static void CleanTimingPanel();

void InterfaceState::UpdateInsights(InterfaceWindow &window)
{
	static int lastPanel = 0;
	int curPanel = 0;
	auto cleanNewlyClosed = [&]()
	{
		if (curPanel == lastPanel) return;
		if (lastPanel == 1)
			CleanTrackingPanel();
		else if (lastPanel == 2)
			CleanTargetCalibrationPanel();
		else if (lastPanel == 3)
			CleanTrackingControllerPanel();
		else if (lastPanel == 4)
			CleanTimingPanel();
		else if (lastPanel == 6)
			CleanSequencePanel();
		lastPanel = curPanel;
	};
	if (!window.open)
	{
		cleanNewlyClosed();
		return;
	}
	if (!ImGui::Begin(window.title.c_str(), &window.open) ||
		!ImGui::BeginTabBar("InsightsSelector", ImGuiTabBarFlags_None))
	{
		cleanNewlyClosed();
		ImGui::End();
		return;
	}

	ServerState &state = GetState();
	PipelineState &pipeline = state.pipeline;

	if (pipeline.phase == PHASE_Tracking && ImGui::BeginTabItem("Tracking"))
	{
		if (ShowTrackingPanel())
			curPanel = 1;
		ImGui::EndTabItem();
	}

	{
		VisTargetLock visTarget = visState.lockVisTarget();
		if ((visTarget.hasObs() || pipeline.phase == PHASE_Calibration_Target)
			&& ImGui::BeginTabItem("Target Calibration"))
		{
			if (ShowTargetCalibrationPanel(visTarget))
				curPanel = 2;
			ImGui::EndTabItem();
		}
	}

	if (ImGui::BeginTabItem("Sequences"))
	{
		if (ShowSequencePanel())
			curPanel = 3;
		ImGui::EndTabItem();
	}

	if (state.mode == MODE_Device && !state.controllers.empty() && ImGui::BeginTabItem("Tracking Controller"))
	{
		if (ShowTrackingControllerPanel())
			curPanel = 4;
		ImGui::EndTabItem();
	}

	if (state.mode == MODE_Device && ImGui::BeginTabItem("Timing"))
	{
		if (ShowTimingPanel())
			curPanel = 5;
		ImGui::EndTabItem();
	}

	cleanNewlyClosed();
	ImGui::EndTabBar();
	ImGui::End();
}

static bool ShowTrackingPanel()
{
	InterfaceState &ui = GetUI();
	ServerState &state = GetState();
	PipelineState &pipeline = state.pipeline;

	int &focusedTrackerID = ui.visState.tracking.focusedTrackerID;
	static int curTrackerID = 0;
	static std::string curTrackerLabel = "Select Tracker";
	if (focusedTrackerID != 0 && focusedTrackerID != curTrackerID)
	{ // External change
		curTrackerID = focusedTrackerID;
		for (auto &tracker : state.trackerConfigs)
		{
			if (tracker.id == focusedTrackerID)
			{
				curTrackerID = tracker.id;
				curTrackerLabel = tracker.label;
			}
		}
	}
	if (ImGui::BeginCombo("Tracker", curTrackerLabel.c_str(), ImGuiComboFlags_WidthFitPreview))
	{ // Local change
		bool changed = false;
		auto TrackerSelect = [&](const TrackerConfig &tracker)
		{
			ImGui::PushID(tracker.id);
			if (ImGui::Selectable(tracker.label.c_str(), curTrackerID == tracker.id) && curTrackerID != tracker.id)
			{
				focusedTrackerID = tracker.id;
				curTrackerID = tracker.id;
				curTrackerLabel = tracker.label;
				changed = true;
			}
			ImGui::PopID();
		};
		for (auto &tracker : state.trackerConfigs)
			TrackerSelect(tracker);
		ImGui::EndCombo();
		if (changed)
			ImGui::MarkItemEdited(ImGui::GetItemID());
	}
	auto trackerIt = std::find_if(state.trackerConfigs.begin(), state.trackerConfigs.end(),
		[&](auto &t){ return t.id == curTrackerID; });
	if (trackerIt == state.trackerConfigs.end()) return false;
	if (trackerIt->type != TrackerConfig::TRACKER_TARGET) return false;
	bool hasIMU = trackerIt->imu != nullptr;
	// TODO: Support marker trackers

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

	auto framesRecord = pipeline.record.frames.getView();
	auto framesStored = state.stored.frames.getView();
	unsigned int frameNum = pipeline.frameNum.load();
	if (framesRecord.empty() && framesStored.empty())
		return false;

	if (!ImPlot::BeginPlot("##RealtimeTracking", ImVec2(-1, -1)))
		return false;

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
	double framesAxisMax = std::max((double)std::max(framesRecord.endIndex(), framesStored.size()), frameRange.Max);

	// Setup plots
	ImPlot::SetupAxis(ImAxis_X1, "Frames",ImPlotAxisFlags_NoLabel);
	ImPlot::SetupAxisLimits(ImAxis_X1, 0, framesAxisMax);
	ImPlot::SetupAxis(ImAxis_Y1, "Samples",ImPlotAxisFlags_Lock);
	ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 50);
	ImPlot::SetupAxis(ImAxis_Y2, "Errors /px", ImPlotAxisFlags_Opposite | ImPlotAxisFlags_Lock);
	ImPlot::SetupAxisLimits(ImAxis_Y2, 0, 1.0f);
	//if (hasIMU)
	{
		ImPlot::SetupAxis(ImAxis_Y3, "IMU /Hz", ImPlotAxisFlags_Opposite | ImPlotAxisFlags_Lock);
		ImPlot::SetupAxisLimits(ImAxis_Y3, 0, 1020);
	}
	// TODO: Would love to toggle time axis along with time data, but needs to be setup before SetAxis is called
	// Thus, the only way to do this is to implement a IsItemHidden with from the last call, and show axis by default
	// String ID doesn't work since something (perhaps SetupFinish) will change the ID stack before the actual item is drawn
	{
		ImPlot::SetupAxis(ImAxis_Y4, "Time", ImPlotAxisFlags_Lock);
		ImPlot::SetupAxisLimits(ImAxis_Y4, 0, 10);
	}
	ImPlot::SetupAxisLinks(ImAxis_X1, &frameRange.Min, &frameRange.Max);

	// Update frameRange from inputs
	ImPlot::SetupFinish(); // Will process inputs, but not update frameRange yet (EndPlot does)
	frameRange = ImPlot::GetPlotLimits(ImAxis_X1).X;
	// TODO: Bit of jelly in tracking graph since there's no true constraints to ensure frameRange.Max == framesAxisMax after input

	static struct {
		std::vector<int> sampleCount;
		std::vector<float> errors2D;
		std::vector<float> tracking;
		std::vector<float> procTimeMS;
		std::vector<float> mistrust;

		void setup(int size)
		{
			sampleCount.clear();
			errors2D.clear();
			tracking.clear();
			procTimeMS.clear();
			mistrust.clear();
			sampleCount.resize(size, 0);
			errors2D.resize(size, NAN);
			tracking.resize(size, NAN);
			procTimeMS.resize(size, NAN);
			mistrust.resize(size, NAN);
		}
	} tracking, recording;
	static std::vector<float> imuSampleTime;
	static std::vector<float> imuSampleRate;

	float mistrustScale = ImPlot::GetPlotLimits(IMPLOT_AUTO, ImAxis_Y2).Y.Max / pipeline.params.track.mistrust.maxMistrust;
	auto updateFrameStats = [&](auto &stats, unsigned int index, FrameRecord &frame)
	{
		auto trackRecord = std::find_if(frame.trackers.begin(), frame.trackers.end(), [&](auto &t){ return t.id == curTrackerID; });
		if (trackRecord == frame.trackers.end())
			return;
		stats.sampleCount[index] = trackRecord->error.samples;
		stats.errors2D[index] = trackRecord->error.mean * PixelFactor;

		// Indices into ImPlotColormap_Dark:
		if (trackRecord->result.isFailure() && trackRecord->result.hasFlag(TrackingResult::REMOVED))
			stats.tracking[index] = 0;
		else if (trackRecord->result.isTracked() && trackRecord->result.hasFlag(TrackingResult::CATCHING_UP))
			stats.tracking[index] = 1;
		else if (trackRecord->result.isTracked() && !trackRecord->result.hasFlag(TrackingResult::CATCHING_UP))
			stats.tracking[index] = 2;
		else if (trackRecord->result.isProbe())
			stats.tracking[index] = 3;
		else if (trackRecord->result.isFailure() && !trackRecord->result.hasFlag(TrackingResult::REMOVED))
			stats.tracking[index] = 4;
		else if (trackRecord->result.isDetected())
			stats.tracking[index] = 7;

		stats.procTimeMS[index] = trackRecord->procTimeMS;
		stats.mistrust[index] = trackRecord->mistrust * mistrustScale;
	};

	double frameShift = 0.0f, frameShiftRec = 0.0f;
	bool drawCur = false, drawRec = false;

	long long min = 1, max = 0;
	if (showCur && !framesRecord.empty())
	{ // Gather stats for realtime tracking data
		min = std::max<long long>(frameRange.Min-1, framesRecord.beginIndex());
		max = std::min<long long>(frameRange.Max+1, framesRecord.endIndex()-1);
		if (max >= min)
		{
			std::size_t frameCnt = max-min+1;
			tracking.setup(frameCnt);
			auto frameIt = framesRecord.pos(max);
			for (int i = 0; i < frameCnt; i++, frameIt--)
			{
				if (!*frameIt || !frameIt->get()->finishedProcessing) continue;
				updateFrameStats(tracking, frameCnt-i-1, *frameIt->get());
			}
			frameShift = (double)min;
			drawCur = frameCnt > 0;
		}
		GetUI().RequestUpdates();
	}

	if (hasIMU)
	{ // Show individual IMU samples at their timestamp, with Y showing deltaT to last sample (hopefully building a smooth line)
		imuSampleTime.clear();
		imuSampleRate.clear();
		auto getIMUSamples = [&](auto samples)
		{
			if (max <= min) return; // Need at least two frames to serve as an anchor
			float frameRate = (max-min)/dtS(framesRecord[min]->time, framesRecord[max]->time);
			long long anchorFrame = min;
			TimePoint_t anchorTime = framesRecord[min]->time -  std::chrono::microseconds(trackerIt->imuCalib.timestampOffsetUS);
			TimePoint_t minTime = anchorTime + std::chrono::microseconds((long long)((frameRange.Min - anchorFrame)/frameRate*1000000));
			TimePoint_t maxTime = anchorTime + std::chrono::microseconds((long long)((frameRange.Max - anchorFrame)/frameRate*1000000));
			auto itBegin = std::lower_bound(samples.begin(), samples.end(), minTime);
			auto itEnd = std::upper_bound(samples.begin(), samples.end(), maxTime);
			if (itBegin == itEnd) return;
			imuSampleTime.reserve(itEnd.index()-itBegin.index());
			imuSampleRate.reserve(itEnd.index()-itBegin.index());
			TimePoint_t last = itBegin->timestamp;
			for (auto it = itBegin; it != itEnd; it++)
			{
				imuSampleTime.push_back(anchorFrame + dtS(anchorTime, it->timestamp)*frameRate);
				imuSampleRate.push_back(1/dtS(last, it->timestamp));
				last = it->timestamp;
			}
		};
		auto &imu = *trackerIt->imu;
		if (imu.isFused)
			getIMUSamples(imu.samplesFused.getView<true>());
		else
			getIMUSamples(imu.samplesRaw.getView<true>());
	}

	if (showRec && state.mode == MODE_Replay && !framesStored.empty())
	{ // Gather stats for recorded tracking data
		min = std::max<long long>(frameRange.Min-1, framesStored.beginIndex());
		max = std::min<long long>(frameRange.Max+1, framesStored.endIndex()-1);
		if (max >= min)
		{
			std::size_t frameCnt = max-min+1;
			recording.setup(frameCnt);
			auto frameIt = framesStored.pos(max);
			for (int i = 0; i < frameCnt; i++, frameIt--)
			{
				if (!*frameIt) continue;
				updateFrameStats(recording, frameCnt-i-1, *frameIt->get());
			}
			frameShiftRec = (double)min;
			drawRec = frameCnt > 0;
		}
	}

	ImPlot::PushColormap(ImPlotColormap_Deep);

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
		ImPlot::PlotBars("Samples", recording.sampleCount.data(), recording.sampleCount.size(), 0.67f, frameShiftRec);
	}

	// Error line
	ImPlot::SetAxis(ImAxis_Y2);
	if (drawRec)
	{ // Draw recorded
		ImPlot::SetNextLineStyle(ImVec4(0.87*0.6, 0.52*0.6, 0.32*0.6, 1.0), 2.0);
		ImPlot::PlotLine("Errors", recording.errors2D.data(), recording.errors2D.size(), 1, frameShiftRec);
	}
	if (drawCur)
	{ // Draw current
		ImPlot::SetNextLineStyle(ImVec4(0.87, 0.52, 0.32, 1), 2.0);
		ImPlot::PlotLine("Errors", tracking.errors2D.data(), tracking.errors2D.size(), 1, frameShift);
	}

	// Processing time line
	ImPlot::SetAxis(ImAxis_Y4); // Use same axis, but not same scala
	if (drawRec)
	{ // Draw recorded
		ImPlot::HideNextItem(ImGuiCond_Appearing);
		ImPlot::SetNextLineStyle(ImVec4(0.8*0.6, 0.2*0.6, 0.8*0.6, 1.0), 2.0);
		ImPlot::PlotLine("Time", recording.procTimeMS.data(), recording.procTimeMS.size(), 1, frameShiftRec);
	}
	if (drawCur)
	{ // Draw current
		ImPlot::HideNextItem(ImGuiCond_Appearing);
		ImPlot::SetNextLineStyle(ImVec4(0.8, 0.2, 0.8, 1), 2.0);
		ImPlot::PlotLine("Time", tracking.procTimeMS.data(), tracking.procTimeMS.size(), 1, frameShift);
	}

	// Mistrust line
	ImPlot::SetAxis(ImAxis_Y2); // Use same axis, but not same scala
	if (drawRec)
	{ // Draw recorded
		ImPlot::SetNextLineStyle(ImVec4(1.0*0.6, 0.2*0.6, 0.2*0.6, 1.0), 2.0);
		ImPlot::PlotLine("Mistrust", recording.mistrust.data(), recording.mistrust.size(), 1, frameShiftRec);
	}
	if (drawCur)
	{ // Draw current
		ImPlot::SetNextLineStyle(ImVec4(1.0, 0.2, 0.2, 1), 2.0);
		ImPlot::PlotLine("Mistrust", tracking.mistrust.data(), tracking.mistrust.size(), 1, frameShift);
	}

	// IMU Update Rate line
	if (hasIMU)
	{ // Draw recorded
		ImPlot::SetAxis(ImAxis_Y3);
		ImPlot::SetNextMarkerStyle(ImPlotMarker_Diamond, 2, ImVec4(0.87, 0.82, 0.22, 1.0), 0);
		ImPlot::PlotScatter("IMU", imuSampleTime.data(), imuSampleRate.data(), imuSampleTime.size());
	}

	// Tracking state
	ImPlot::PushColormap(ImPlotColormap_Dark);
	if (drawRec)
		ImPlot::PlotDigital("##TrackingRec", recording.tracking.data(), recording.tracking.size(), frameShiftRec-0.5f);
	if (drawCur)
		ImPlot::PlotDigital("##Tracking", tracking.tracking.data(), tracking.tracking.size(), frameShift-0.5f);
	ImPlot::PopColormap();

	// Current and selected frames
	double curFrame = frameNum + 0.5;
	double jumpFrame = (double)ui.frameJumpTarget;
	ImPlot::SetNextLineStyle(ImVec4(0.33, 0.66, 0.4, 1.0), 3);
	ImPlot::PlotInfLines("CurFrame", &curFrame, 1);
	if (ImPlot::DragLineX(2, &jumpFrame, ImVec4(0.66, 0.33, 0.4, 1.0), 3))
		ui.frameJumpTarget = (unsigned int)jumpFrame;

	ImPlot::PopColormap();
	ImPlot::EndPlot();
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

			ImPlot::SetupAxis(ImAxis_Y1, "Markers",ImPlotAxisFlags_LockMin | ImPlotAxisFlags_AutoFit);
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

	if (visTarget.hasObs() && ImGui::BeginTabItem(targetMarkerLabel))
	{
		curTab = targetMarkerLabel;

		ui.visState.updateVisTarget(visTarget);

		if (!ui.seqTarget || ui.seqTarget->needsUpdate(visTarget))
		{ // Need to update/recreate sequence
			ui.seqTarget = make_opaque<TargetViewSequence>(visTarget, ui.visState, pipeline.getCalibs());

			// Keep zoom intact
			ui.seqTarget->framePixelWidthTarget = targetViewZoom;
			ui.seqTarget->framePixelWidth = targetViewZoom;
		}
		targetViewZoom = ui.seqTarget->framePixelWidthTarget;

		ImSequencer::Sequencer(ui.seqTarget.get(), &ui.visState.targetCalib.frameIdx, nullptr, &ui.visState.target.markerFocussed, &ui.seqTarget->m_start,
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

static bool ShowSequencePanel()
{
	InterfaceState &ui = GetUI();
	ServerState &state = GetState();
	PipelineState &pipeline = state.pipeline;

	static float sequenceZoom = 10.0f;

	static bool followFrame = true;
	ImGui::Checkbox("Auto-Size", &followFrame);

	if (!ui.seqMarker || ui.seqMarker->needsUpdate(*pipeline.seqDatabase.contextualRLock()))
	{ // Need to update/recreate sequence
		auto lock = folly::detail::lock(folly::detail::wlock(pipeline.calibration), folly::detail::wlock(pipeline.seqDatabase));
		ui.seqMarker = make_opaque<Marker2DSequence>(*std::get<1>(lock), ui.visState, *std::get<0>(lock));

		// Keep zoom intact
		ui.seqMarker->framePixelWidthTarget = sequenceZoom;
		ui.seqMarker->framePixelWidth = sequenceZoom;
	}
	if (followFrame)
	{
		ui.seqMarker->framePixelWidthTarget = ui.seqMarker->framePixelWidth =
			(float)(ImGui::GetContentRegionAvail().x - 200 - 36) // Why -36? Idk ImSequencer is weird
				/ (ui.seqMarker->GetFrameMax() - ui.seqMarker->GetFrameMin());
	}
	sequenceZoom = ui.seqMarker->framePixelWidthTarget;

	int curFrame = pipeline.frameNum;
	ImSequencer::Sequencer(ui.seqMarker.get(), &curFrame, nullptr, nullptr, &ui.seqMarker->m_start, ImSequencer::SEQUENCER_EDIT_NONE);

	return true;
}
static void CleanSequencePanel()
{
	InterfaceState &ui = GetUI();
	if (ui.seqMarker)
	{
		ui.seqMarker = nullptr;
	}
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

	if (ImGui::TabItemButton(ui.seqEventsActive? "Disable##Toggle" : "Enable##Toggle", ImGuiTabItemFlags_Trailing | ImGuiTabItemFlags_NoTooltip))
	{
		ui.seqEventsActive = !ui.seqEventsActive;
		if (ui.seqEventsActive)
			comm_submit_control_data(controller->comm, COMMAND_OUT_EVENTS, 0x00, eventClassCode, nullptr, 0);
		else
			comm_submit_control_data(controller->comm, COMMAND_OUT_EVENTS, 0x00, 0, nullptr, 0);
	}

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

static int PlotFormatterTimeUS(double value, char* buff, int size, void* data)
{
	ImAxis axis = (ImAxis)(intptr_t)data;
	double width = 0;
	if (axis < ImAxis_Y1)
		width = ImPlot::GetPlotLimits(axis, IMPLOT_AUTO).Size().x;
	else
	 	width = ImPlot::GetPlotLimits(IMPLOT_AUTO, axis).Size().y;
	long timeUS = (long)std::max((double)LONG_MIN, std::min((double)LONG_MAX, value));
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
}
static bool ShowTimeSyncPanel(BlockedQueue<TimeSyncMeasurement, 4096>::View<true> &samples)
{
	InterfaceState &ui = GetUI();
	ServerState &state = GetState();

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

	static TimeSyncParams params = TimeSyncParamsForUSB;
	static bool emulate = false;
	updateGraph |= ImGui::Checkbox("Enable Emulation", &emulate);
	if (emulate)
	{
		updateGraph |= ImGui::SliderFloat("Drift Lerp", &params.driftLerp, 0, 0.00005f, "%.10ff", ImGuiSliderFlags_NoRoundToFormat);
		updateGraph |= ImGui::SliderFloat("Drift Bias", &params.driftBias, -0.00001f, 0.00001f, "%.10ff", ImGuiSliderFlags_NoRoundToFormat);
		updateGraph |= ImGui::InputInt("Drift Init Range", &params.driftInitRange, 0, 10000);
		updateGraph |= ImGui::SliderFloat("Drift Init Adapt", &params.driftInitAdapt, 0.0f, 1000.0f);
		updateGraph |= ImGui::SliderFloat("Drift Downward Correct", &params.driftDownwardCorrect, 0.0f, 1000.0f);
		updateGraph |= ImGui::SliderFloat("Drift Downward Jump", &params.driftDownwardJump, 0.0f, 1.0f);
		updateGraph |= ImGui::InputInt("Offset US", &params.timeOffsetUS);
		updateGraph |= ImGui::InputInt("Max Lerp Diff US", &params.maxLerpDiffUS);
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

		ImPlot::SetupAxis(ImAxis_X1, "Base Timestamp", ImPlotAxisFlags_AutoFit);
		ImPlot::SetupAxisFormat(ImAxis_X1, PlotFormatterTimeUS, (void*)(intptr_t)ImAxis_X1);
		ImPlot::SetupAxis(ImAxis_Y1, "Drift / Latency", ImPlotAxisFlags_AutoFit);
		ImPlot::SetupAxisFormat(ImAxis_Y1, PlotFormatterTimeUS, (void*)(intptr_t)ImAxis_Y1);

		static std::vector<double> sourceTimestamps;
		static std::vector<double> measuredTimesOffset;
		static std::vector<double> estimatedTimesOffset;
		static std::vector<double> emulatedTimesOffset;
		static std::vector<double> resetEventX;
		static std::vector<double> resetEventY;

		if (updateGraph)
		{
			sourceTimestamps.clear();
			measuredTimesOffset.clear();
			estimatedTimesOffset.clear();
			emulatedTimesOffset.clear();
			resetEventX.clear();
			resetEventY.clear();

			std::size_t sampleCount = samples.endIndex()-samples.beginIndex();
			sourceTimestamps.reserve(sampleCount);
			measuredTimesOffset.reserve(sampleCount);
			estimatedTimesOffset.reserve(sampleCount);
			emulatedTimesOffset.reserve(sampleCount);

			TimeSync emulatedTimeSync = { params };

			for (const auto &sample : samples)
			{
				long long baseUS = sample.timestamp - referenceTimestamp;
				sourceTimestamps.push_back((double)baseUS);
				measuredTimesOffset.push_back((double)(dtUS(referenceTime, sample.measurement) - baseUS));
				estimatedTimesOffset.push_back((double)(dtUS(referenceTime, sample.estimation) - baseUS));

				if (emulate)
				{
					uint64_t overflow = (uint64_t)1<<63;
					uint64_t timestamp = RebaseSourceTimestamp(emulatedTimeSync, sample.timestamp&(overflow-1), overflow, sample.measurement);
					TimePoint_t emulated = UpdateTimeSync(emulatedTimeSync, timestamp, sample.measurement);
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
		ImPlot::PlotScatter("Measurements", sourceTimestamps.data(), measuredTimesOffset.data(), sourceTimestamps.size(), ImPlotLineFlags_None);
		ImPlot::PlotScatter("Estimations", sourceTimestamps.data(), estimatedTimesOffset.data(), sourceTimestamps.size(), ImPlotLineFlags_None);
		if (emulate)
		{
			ImPlot::PlotScatter("Emulated Estimations", sourceTimestamps.data(), emulatedTimesOffset.data(), sourceTimestamps.size(), ImPlotLineFlags_None);
			ImPlot::PlotScatter("Emulated Resets", resetEventX.data(), resetEventY.data(), resetEventX.size(), ImPlotLineFlags_None);
		}
		ImPlot::PopStyleVar();

		ImPlot::EndPlot();
	}

	return true;
}
static bool ShowLatencyPanel(BlockedQueue<LatencyMeasurement, 4096>::View<true> &samples, const std::vector<std::string> &descriptions)
{
	InterfaceState &ui = GetUI();
	ServerState &state = GetState();

	static bool followFrame = true;
	ImGui::SameLine(); // Append to header
	ImGui::Checkbox("Follow Frame", &followFrame);

	static std::size_t graphBegin = 0, graphEnd = 0;
	static TimePoint_t timeBegin, timeEnd;
	static double timeSpan;
	bool updateGraph = graphBegin != samples.beginIndex() || graphEnd != samples.endIndex();
	std::size_t sampleCount = samples.endIndex() - samples.beginIndex();
	if (updateGraph)
	{
		graphBegin = samples.beginIndex();
		graphEnd = samples.endIndex();
		if (!samples.empty())
		{
			timeBegin = samples.front().sample;
			timeEnd = samples.back().sample;
			timeSpan = dtUS(timeBegin, timeEnd);
		}
	}

	if (ImPlot::BeginPlot("##Latency", ImVec2(-1, -1)))
	{
		// Update frameRange
		static ImPlotRange frameRange(0, 1000000);
		int maxOffset = std::max(1000.0, frameRange.Size()/6);
		if (followFrame)
		{ // Apply offset due to new recent frame
			double offset = 0;
			if (frameRange.Max < timeSpan+maxOffset)
				offset = std::max(timeSpan + maxOffset - frameRange.Max, -frameRange.Min);
			else if (frameRange.Min > timeSpan-maxOffset)
				offset = std::max(timeSpan - maxOffset - frameRange.Min, -frameRange.Min);
			frameRange.Min += offset;
			frameRange.Max += offset;
		}
		double framesAxisMax = std::max(timeSpan, frameRange.Max);

		// Setup plot
		ImPlot::SetupAxis(ImAxis_X1, "Sample Timestamp (us)");
		ImPlot::SetupAxisFormat(ImAxis_X1, PlotFormatterTimeUS, (void*)(intptr_t)ImAxis_X1);
		ImPlot::SetupAxisLimits(ImAxis_X1, 0, framesAxisMax);
		ImPlot::SetupAxisLinks(ImAxis_X1, &frameRange.Min, &frameRange.Max);
		ImPlot::SetupAxis(ImAxis_Y1, "Latency (us)", ImPlotAxisFlags_LockMin);
		ImPlot::SetupAxisFormat(ImAxis_Y1, PlotFormatterTimeUS, (void*)(intptr_t)ImAxis_Y1);
		ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 50000);

		// Update frameRange from inputs
		ImPlot::SetupFinish(); // Will process inputs, but not update frameRange yet (EndPlot does)
		frameRange = ImPlot::GetPlotLimits(ImAxis_X1).X;

		static std::vector<double> sampleTimesstamps;
		static std::vector<double> latencyOffsets[LatencyStackSize];

		if (updateGraph)
		{
			sampleTimesstamps.clear();
			for (auto &latencyOffset : latencyOffsets)
				latencyOffset.clear();

			sampleTimesstamps.reserve(sampleCount);
			for (int i = 0; i < descriptions.size(); i++)
				latencyOffsets[i].reserve(sampleCount);

			for (const auto &sample : samples)
			{
				sampleTimesstamps.push_back((double)dtUS(timeBegin, sample.sample));
				for (int i = 0; i < descriptions.size(); i++)
					latencyOffsets[i].push_back((double)sample.latency[i]);
			}
		}

		ImPlot::PushStyleVar(ImPlotStyleVar_MarkerSize, 2);
		for (int i = descriptions.size()-1; i >= 0; i--)
		{
			ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle);
			ImPlot::PlotStems(descriptions[i].c_str(), sampleTimesstamps.data(), latencyOffsets[i].data(), sampleCount);
		}

		ImPlot::PopStyleVar();

		ImPlot::EndPlot();
	}

	return true;
}
static BlockedQueue<TimeSyncMeasurement, 4096> recordedTimesync;
static bool ShowTimingPanel()
{
	InterfaceState &ui = GetUI();
	ServerState &state = GetState();

	enum class TimeType { None = -1, TimeSync, Latency, Max };
	const char* timeTypeSelection[] = { "TimeSync", "Latency" };
	static TimeType timeType = TimeType::None;
	static int sourceType = -1;
	static int sourceIndex = -1;
	static std::string sourceLabel;

	static std::vector<std::string> recordedTimesyncs;
	auto getRecordingLabel = [](std::string &record)
	{
		std::size_t pos = record.find_last_of('/');
		pos = (pos == std::string::npos)? 0 : pos+1;
		return asprintf_s("Record '%s'", record.c_str()+pos);
	};
	auto loadTimeSyncRecording = [&getRecordingLabel](int index)
	{
		std::ifstream ifs(recordedTimesyncs[index]);
		if (!ifs.is_open()) return;
		recordedTimesync.cull_clear();
		sourceType = 3;
		sourceIndex = index;
		sourceLabel = getRecordingLabel(recordedTimesyncs[index]);
		TimePoint_t startReference = sclock::now();
		std::string line;
		while (std::getline(ifs, line))
		{
			uint64_t timestamp, receiveTimeEst, roundtripUS, timeSinceFrameUS; // Last two are optional
			int cnt = std::sscanf(line.c_str(), "%lu, %lu, %lu, %lu", &timestamp, &receiveTimeEst, &roundtripUS, &timeSinceFrameUS);
			if (cnt >= 2)
			{
				TimeSyncMeasurement meas;
				meas.timestamp = timestamp;
				meas.measurement = startReference + std::chrono::microseconds(receiveTimeEst);
				meas.estimation = meas.measurement; // No estimation
				recordedTimesync.push_back(std::move(meas));
			}
		}
	};

	static const char* typeLabel;
	if (timeType == TimeType::None) typeLabel = "Select Type";
	if (ImGui::BeginCombo("Type", typeLabel, ImGuiComboFlags_WidthFitPreview))
	{
		bool changed = false;
		for (int i = 0; i < (int)TimeType::Max; i++)
		{
			if (!ImGui::Selectable(timeTypeSelection[i])) continue;
			changed = true;
			typeLabel = timeTypeSelection[i];
			timeType = (TimeType)i;
		}
		ImGui::EndCombo();
		if (changed)
			ImGui::MarkItemEdited(ImGui::GetItemID());
	}
	if (timeType == TimeType::None) return false;

	ImGui::SameLine();
	if (sourceType < 0) sourceLabel = "Select Source";
	if (ImGui::BeginCombo("Source", sourceLabel.c_str(), ImGuiComboFlags_WidthFitPreview))
	{
		bool changed = false;
		for (int i = 0; i < state.controllers.size(); i++)
		{
			std::string label = asprintf_s("Controller %d", i);
			if (!ImGui::Selectable(label.c_str(), sourceType == 0 && sourceIndex == i)) continue;
			changed = true;
			sourceType = 0;
			sourceIndex = i;
			sourceLabel = std::move(label);
		}
		auto imuProviders = state.imuProviders.contextualRLock();
		for (int i = 0; i < imuProviders->size(); i++)
		{
			auto &imuProvider = imuProviders->at(i);
			if (!imuProvider) continue;
			std::string label = imuProvider->getDescriptor();
			if (!label.empty())
			{
				bool supported = (timeType == TimeType::TimeSync && imuProvider->timingRecord.supportsTimeSync)
							  || (timeType == TimeType::Latency && imuProvider->timingRecord.supportsLatency);
				if (supported && ImGui::Selectable(label.c_str() , sourceType == 2 && sourceIndex == i))
				{
					changed = true;
					sourceType = 2;
					sourceIndex = i;
					sourceLabel = std::move(label);
				}
				else if (!supported)
					ImGui::Text("%s", label.c_str());
				ImGui::Indent();
			}

			for (auto &imu : imuProvider->devices)
			{
				if (imu->id.driver == IMU_DRIVER_NONE) continue; // Not an IMUDevice
				IMUDevice &device = *(IMUDevice*)imu.get();
				bool supported = (timeType == TimeType::TimeSync && device.timingRecord.supportsTimeSync)
							  || (timeType == TimeType::Latency && device.timingRecord.supportsLatency);
				if (!supported) continue;
				std::string label = asprintf_s("%s (%d) [%d]", device.getDescriptor().c_str(), device.id.driver, device.index);
				if (label.empty()) continue;
				if (!ImGui::Selectable(label.c_str() , sourceType == 1 && sourceIndex == device.index)) continue;
				changed = true;
				sourceType = 1;
				sourceIndex = device.index;
				sourceLabel = std::move(label);
			}

			if (!label.empty())
				ImGui::Unindent();
		}
		if (timeType == TimeType::TimeSync)
		{
			for (int i = 0; i < recordedTimesyncs.size(); i++)
			{
				if (!ImGui::Selectable(getRecordingLabel(recordedTimesyncs[i]).c_str(), sourceType == 3 && sourceIndex == i)) continue;
				loadTimeSyncRecording(i);
				changed = true;
			}
		}
		ImGui::EndCombo();
		if (changed)
			ImGui::MarkItemEdited(ImGui::GetItemID());
	}

	ImGui::SameLine();
	if (ImGui::Button("Open Recording"))
	{
		threadPool.push([&loadTimeSyncRecording](int id)
		{
			if (!NFD_Init())
			{ // Thread-specific init
				SignalErrorToUser(asprintf_s("Failed to initialise File Picker: %s", NFD_GetError()));
				return;
			}

			const int filterLen = 1;
			nfdfilteritem_t filterList[filterLen] = {

				{"Camera TimeSync Recording", "csv"},
			};
			nfdchar_t *outPath;
		#ifdef _WIN32
			std::string defPath = "Downloads";
		#else
			std::string defPath = std::filesystem::current_path();
		#endif
			nfdopendialogu8args_t args;
			args.filterList = filterList;
			args.filterCount = filterLen;
			args.defaultPath = defPath.c_str();
			NFD_GetNativeWindowFromGLFWWindow(GetUI().glfwWindow, &args.parentWindow);
			nfdresult_t result = NFD_OpenDialogU8_With(&outPath, &args);
			if (result == NFD_OKAY)
			{
				recordedTimesyncs.push_back(std::string(outPath));
				loadTimeSyncRecording(recordedTimesyncs.size()-1);
				NFD_FreePath(outPath);
			}
			else if (result == NFD_ERROR)
			{
				SignalErrorToUser(asprintf_s("Failed to use File Picker: %s", NFD_GetError()));
			}

			NFD_Quit();

			GetUI().RequestUpdates();
		});
	}

	// Update all available TimingRecord sources and find selected
	BlockedQueue<TimeSyncMeasurement, 4096>::View<true> timesyncView;
	BlockedQueue<LatencyMeasurement, 4096>::View<true> latencyView;
	const std::vector<std::string> *latencyDescriptions = nullptr;
	bool found = false;
	auto handleTimingRecord = [&](TimingRecord &record, bool selected)
	{
		record.recordTimeSync = selected && timeType == TimeType::TimeSync && record.supportsTimeSync;
		record.recordLatency = selected && timeType == TimeType::Latency && record.supportsLatency;
		if (record.recordTimeSync)
		{ // Blocks of 4096 measurements each - keep only last 10
			record.timeSync.delete_culled();
			record.timeSync.cull_front(-10);
			timesyncView = record.timeSync.getView();
		}
		else if (record.recordLatency)
		{ // Blocks of 4096 measurements each - keep only last 10
			record.latency.delete_culled();
			record.latency.cull_front(-10);
			latencyView = record.latency.getView();
			latencyDescriptions = &record.latencyDescriptions;
		}
		else return false;
		found = true;
		return true;
	};

	for (int i = 0; i < state.controllers.size(); i++)
	{
		if (!handleTimingRecord(state.controllers[i]->timingRecord, sourceType == 0 && sourceIndex == i)) continue;
		if (state.controllers[i]->comm->commStreaming)
			ui.RequestUpdates();
	}
	for (auto &imu : state.pipeline.record.imus)
	{
		if (imu->id.driver == IMU_DRIVER_NONE) continue; // Not an IMUDevice
		IMUDevice &device = *(IMUDevice*)imu.get();
		if (!handleTimingRecord(device.timingRecord, sourceType == 1 && sourceIndex == device.index)) continue;
		ui.RequestUpdates();
	}
	auto imuProviders = state.imuProviders.contextualRLock();
	for (int i = 0; i < imuProviders->size(); i++)
	{
		if (!imuProviders->at(i)) continue;
		if (!handleTimingRecord(imuProviders->at(i)->timingRecord, sourceType == 2 && sourceIndex == i)) continue;
	}
	if (timeType == TimeType::TimeSync && sourceType == 3 && sourceIndex >= 0)
	{
		timesyncView = recordedTimesync.getView();
		found = true;
	}
	else
	{
		recordedTimesync.cull_clear();
		recordedTimesync.delete_culled();
	}

	if (!found) return false; // Influences if we get to cleanup after panel gets inactive
	imuProviders.unlock();
	ui.RequestUpdates();

	if (timeType == TimeType::TimeSync)
		return ShowTimeSyncPanel(timesyncView);
	else if (timeType == TimeType::Latency && !latencyDescriptions->empty())
		return ShowLatencyPanel(latencyView, *latencyDescriptions);
	return true;
}
static void CleanTimingPanel()
{
	for (auto &controller : GetState().controllers)
		controller->timingRecord.clear();
	for (auto &imu : GetState().pipeline.record.imus)
	{
		if (imu->id.driver == IMU_DRIVER_NONE) continue; // Not an IMUDevice
		IMUDevice &device = *(IMUDevice*)imu.get();
		device.timingRecord.clear();
	}
	auto imuProviders = GetState().imuProviders.contextualRLock();
	for (int i = 0; i < imuProviders->size(); i++)
	{
		if (!imuProviders->at(i)) continue;
		imuProviders->at(i)->timingRecord.clear();
	}
	recordedTimesync.clear();
}