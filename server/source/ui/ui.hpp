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

#ifndef UI_H
#define UI_H

/**
 * Shared between all ui compile units, not a header for external use
 */

#include "server.hpp"

#ifndef IMGUI_DEFINE_MATH_OPERATORS
#define IMGUI_DEFINE_MATH_OPERATORS
#include "imgui.h"
#endif
#include "imgui/imgui_internal.h"
#include "imgui/imgui_custom.hpp"
#include "imgui/misc/cpp/imgui_stdlib.h"

#include "gl/visualisation.hpp"

#include "util/eigenutil.hpp"
#include "util/util.hpp" // TimePoint_t
#include "util/blocked_vector.hpp"
#include "util/synchronised.hpp"
#include "util/memory.hpp"
#include "util/log.hpp"

#include <map>

// Forward-declared opaque structs
struct TrackingCameraState; // device/tracking_camera.hpp
struct TrackingControllerState; // device/tracking_controller.hpp
struct CameraConfig; // config.hpp
struct TargetViewSequence; // insights.hpp
struct TrackingControllerEventSequence; // insights.hpp
struct TargetView; // calib_target/assembly.hpp
struct TargetAssemblyStage; // calib_target/assembly.hpp
struct ObsTarget; // calib/obs_data.hpp
struct ObsTargetFrame; // calib/obs_data.hpp
struct SequenceData; // point/sequence_data.hpp
struct VisFrameLock; // ui/system/vis.hpp
struct VisTargetLock; // ui/system/vis.hpp
struct Color; // gl/visualisation.hpp
struct GLFWwindow; // GLFW/glfw3.h
// Defined later
struct CameraView;
class InterfaceState;

extern InterfaceState *InterfaceInstance;
static inline InterfaceState &GetUI() { return *InterfaceInstance; }

struct IconSet
{
	ImTextureID wireless, server, frameWireless, frameHDMI, visual, detach, context, controller, camera;
};

struct InterfaceWindow
{
	std::string title;
	ImGuiID id;
	bool open;
	void (InterfaceState::*updateWindow)(InterfaceWindow&);

	InterfaceWindow() {}
	InterfaceWindow(const char* Title, void (InterfaceState::*UpdateWindow)(InterfaceWindow&), bool Open = false)
		: title(Title), id(ImHashStr(Title)), updateWindow(UpdateWindow), open(Open) {}
};

enum InterfaceWindows
{
	WIN_3D_VIEW,
	WIN_CAMERA_VIEWS,
	WIN_VISUALISATION,

	WIN_PIPELINE,
	WIN_CONTROL,

	WIN_LOGGING,
	WIN_INSIGHTS,

	WIN_TARGETS,

	WIN_DEVICES,
	WIN_CAMERA_SETTINGS,
	WIN_WIRELESS,

	WIN_SEQUENCE_PARAMS,
	WIN_POINT_CALIB_PARAMS,
	WIN_TARGET_CALIB_PARAMS,
	WIN_TRACKING_PARAMS,

	WIN_LENS_SELECTION_TOOL,

	WIN_STYLE_EDITOR,
	WIN_IMGUI_DEMO,
	WIN_IMPLOT_DEMO,
	INTERFACE_WINDOWS_MAX
};

struct VisualisationState
{
	VisFrameLock lockVisFrame(const PipelineState &pipeline, bool forceRealtime = false) const;
	Eigen::Vector3f getPreferredTarget(const VisFrameLock &visFrame) const;

	bool showMarkerTrails = false;
	bool showMarkerRays = false;

	struct
	{
		float brightness = 0.0f;
		float contrast = 1.0f;
	} image;

	struct
	{
		bool showFoVCircle, showFoVBounds, showDesignCalib;
		float circularFoV = 90.0f;
		std::array<float,3> boundsFoV = { 80, 60, 100 };
		std::vector<CameraCalib> designCalibs;
	} calib;

	struct
	{
		// Tracking target to focus on		
		int focusedTargetID = 0;
		struct TrackingTargets
		{
			const TargetCalibration3D * calib = nullptr;
			long lastFrame;
			std::string imu;
		};
		std::map<int, TrackingTargets> targets;
		TargetTracking2DData retrackData;

		// Tracking visualisations
		bool showOrphanedIMUs = true;
		bool showSearchBounds = false;
		bool showTargetObserved = true, showTargetPredicted = false, showTargetFiltered = true, showTargetFilteredCamera = false;
		bool showPoseExtrapolated = false, showInertialIntegrated = false, showInertialFused = false, showInertialFiltered = false;
		int trailLength = 0;
		bool showCovariancePos = false, showCovarianceRot = false;
		float scaleCovariance = 10.0f;

		// Debug visualisation settings & state
		bool showUncertaintyAxis = false;
		bool debugMatchingState = true;
		int debugFocusStage = 0;
		int debugFocusPoint = -1;
		bool onlyFocusPoint = false;
		bool showAllLabels = false;
		struct
		{
			int frameNum = -1, trackerID = 0;
			bool needsUpdate = false;
			TargetMatch2D targetMatch2D;
			TargetMatch2D editedMatch2D;
			bool showEdited = false;
			bool showEditTools = false;
			std::vector<std::vector<SceneLabel>> priLabels;
			std::vector<std::vector<SceneLabel>> secLabels;
			std::vector<std::vector<SceneButton>> editButtons;
		} debug;
	} tracking;

	struct
	{
		struct ObservationCompare
		{
			std::vector<BlockedVector<Eigen::Vector2f>> visPoints;
			int markerCount, obsCount, triCount;
		};
		std::vector<ObservationCompare> savedObs;
		int visSavedObs = -1;
	} observations;

	struct
	{
		// Selection of target for inspection outside of target calib
		int selectedTargetID = 0;
		TargetCalibration3D selectedTargetCalib;

		// Visualisation
		std::vector<bool> cameraRays;
		std::vector<bool> markerSelect;
		bool focusOnMarkerSelection = false;
		int markerFocussed = -1; // Hovered in "Insights/Target Markers" Sequencer
		int markerHovered = -1; // Hovered in 3D View
		bool markerObservations = false;
		bool markerViewCones = true;
	} target;

	// Target Calibration
	struct
	{
		// Currently edited assembly stage
		std::shared_ptr<TargetAssemblyBase> edit = nullptr;
		int highlightedSequence = -1, selectedSequence = -1;
		// Selection of existing target views or assembly stages
		std::shared_ptr<TargetView> view = nullptr;
		std::shared_ptr<TargetAssemblyStage> stage = nullptr;
		int stageSubIndex = -1, stageSubSubIndex = -1;
		// Frame within VisTargetLock
		int frameIdx = 0, frameNum = 0;
	} targetCalib;
	VisTargetLock lockVisTarget() const;
	bool resetVisTarget(bool keepFrame = true);
	void updateVisTarget(const VisTargetLock &visTarget);
	void updateVisTarget();

	// Incremental update of observations
	struct {
		// Not synchronised, only updated by UpdateIncrementalObservationVis
		// And read by UI right after
		int frameStable = 0; // Last time the stable list was updated
		int pointsStable = 0, pointsUnstable = 0;
		std::vector<int> cameraTriObservations;
		int markerCount = 0; // To check if observations were cleared
		bool dirty = false;
	} incObsUpdate;

	// Visualisation of reference points in virtual space
	struct
	{
		// TODO: Add other cameras, floorplane once calibrated, etc.
		bool showOrigin = false;
		Eigen::Vector3f origin = Eigen::Vector3f::Zero();
	} room;
};

struct View3D
{
	// General projection
	float fov = 65.0f;
	float pitch = 0, heading = 0;
	Eigen::Isometry3f viewTransform;

	// Orbit view
	bool orbit;
	float distance;
	Eigen::Vector3f target;

	// Interaction
	bool isDragging = false;
	bool sidePanelOpen = true;
	Eigen::Vector2f mousePos;

	inline Eigen::Projective3f getProj(float aspect) const
	{
		Eigen::Projective3f proj;
		const float zNear = 0.0001f/2, zFar = 100; // 0.1mm-100m
		const float a = -(zFar+zNear)/(zFar-zNear), b = (2*zNear*zFar)/(zFar-zNear);
		float sY = 1.0f / fInvFromFoV(fov);
		float sX = sY*aspect;
		// Projection - negative z because of blender-style camera coordinate system
		proj.matrix() <<
			sX, 0, 0, 0,
			0, sY, 0, 0,
			0, 0, a, b,
			0, 0, 1, 0;
		return proj;
	}
};

/**
 * Interface for the AsterTrack application
 */
class InterfaceState
{
public:
	bool init = false;

	// Window and Platform state
	GLFWwindow *glfwWindow = nullptr;
	bool useCustomHeader = false;
	bool loadedSettingsIni = false;
	bool setCloseInterface = false;

	// Render state
	TimePoint_t renderTime;
	float deltaTime;
	int requireUpdates = 3;
	bool requireRender;

	// UI Theme
	bool isDarkMode = true;
	IconSet lightModeIcons, darkModeIcons;
	IconSet &icons() { return isDarkMode? darkModeIcons : lightModeIcons; }

	// Window state
	ImGuiID dockspaceID;
	std::array<InterfaceWindow, INTERFACE_WINDOWS_MAX> windows;

	// General visualisation state
	VisualisationState visState = {};

	// Camera Views state
	std::map<CameraID, CameraView> cameraViews;
	bool cameraGridDirty = false;

	// 3D View state
	View3D view3D = {};

	// Control/Recording state
	struct RecordedSections
	{
		int begin, end;
		bool forceSave, saved;
		std::string path;
	};
	std::vector<RecordedSections> recordSections;
	long recordSectionStart = -1;
	bool saveTrackingResults = true;
	unsigned int frameJumpTarget = 0;
	bool frameRelevantParametersDirty = false;

	// Sequences state
	opaque_ptr<TargetViewSequence> seqTarget = NULL;
	opaque_ptr<TrackingControllerEventSequence> seqEvents = NULL;
	bool seqEventsActive = false;
	long seqJumpToFrame = -1;

	// Pipeline/PointCalib state
	bool newCalibration = false;
	Synchronised<std::string> calibSamples;
	struct {
		int numCalibrated, numUncalibrated, relCertain, relUncertain;
	} calibState;

	// Pipeline/TargetCalib state
	std::vector<std::shared_ptr<TargetView>> targetViewsSorted;
	bool targetViewsDirty = false;
	int targetDiscardAfterStage = -1;
	bool targetFramesAdvancing = false;

	// Log state
	BlockedVector<std::size_t> logsFiltered;
	std::size_t logsFilterPos = 0;
	bool logsStickToNew = true;

	InterfaceState() { InterfaceInstance = this; }
	~InterfaceState() { if (InterfaceInstance == this) InterfaceInstance = NULL; }

	bool Init();
	void Exit();

	void UpdateSequences(bool reset = false);
		void UpdateIncrementalSequencesVis(const SequenceData &sequences, bool updateStable, bool rawPoints);
	void UpdateCalibrations(bool calibrated = true);

	// UI-only device functions
	CameraConfig& getCameraConfig(const TrackingCameraState &camera);
	std::string getStatusText(const TrackingCameraState &camera);
	Color getStatusColor(const TrackingCameraState &camera);
	Color getStatusColor(const TrackingControllerState &controller);

	void UpdateCameras();
	void ResetCameras();

	void RequestUpdates(int count = 1);
	void RequestRender();
	void UpdateUI();
	void RenderUI(bool fullUpdate = true);
	void ResetWindowLayout();

	void UpdateMainMenuBar();

	void Update3DViewUI(InterfaceWindow &window);
	void UpdateCameraViews(InterfaceWindow &window);
		void UpdateDetachedCameraView(CameraView &camera);
		void UpdateCameraUI(CameraView &camera);
		bool UpdateCameraGrid();
	void UpdateVisualisationSettings(InterfaceWindow &window);

	void UpdatePipeline(InterfaceWindow &window);
		void UpdatePipelineCalibSection();
		void UpdatePipelineObservationSection();
		void UpdatePipelinePointCalib();
		void UpdatePipelineTargetCalib();
	void UpdateControl(InterfaceWindow &window);

	void UpdateLogging(InterfaceWindow &window);
	void UpdateInsights(InterfaceWindow &window);

	void UpdateTargets(InterfaceWindow &window);

	void UpdateDevices(InterfaceWindow &window);
	void UpdateCameraSettings(InterfaceWindow &window);
	void UpdateWirelessSetup(InterfaceWindow &window);

	void UpdateSequenceParameters(InterfaceWindow &window);
	void UpdatePointCalibParameters(InterfaceWindow &window);
	void UpdateTargetCalibParameters(InterfaceWindow &window);
	void UpdateTrackingParameters(InterfaceWindow &window);

	void UpdateLensSelectionTool(InterfaceWindow &window);

	void UpdateStyleUI(InterfaceWindow &window);
	void UpdateImGuiDemoUI(InterfaceWindow &window);
	void UpdateImPlotDemoUI(InterfaceWindow &window);

	void GeneralInput();
	void GeneralUpdate();
};


/*
 * Camera View
 */

enum CameraVisMode
{
	VIS_BLOB,
	VIS_BACKGROUND_TILES,
	VIS_VISUAL_DEBUG,
	VIS_ERROR_VIS,
	VIS_EMULATION,
	//VIS_CAMERA_FRAME
};

struct BlobEmulationResults;
struct BlobEmulationVis;

struct CameraVisState
{
	CameraVisMode visMode;

	struct {
		bool rotate180 = true; // Needs to be rotated when hanging from ceiling
		Eigen::Vector2f center = Eigen::Vector2f::Zero();
		Eigen::Vector2f prevCenter;
		bool isDragging; // Manually changing target2D
		float zoom = 1.0f;
		float prevZoom;
		float camZoom = 1.0f;
		float maxZoom = 1.0f;
		bool autoZoom; // Select target2D automatically based on phase
		int autoZoomDelay;
	} view = {};

	struct {
		bool show = false;
		bool showBlobs = true;
		bool syncVis = true;
		bool undistort = true;
		bool followFrame = true;
		unsigned int texID;
	} imageVis = {};

	std::shared_ptr<const CameraImage> image;

	struct {
		bool enabled, update;
		// Options for visualistion
		struct {
			bool maskAll, maskEdge, maskCenter;
			bool blobCam = false, blobEmulated = true;
			bool blobRefined, edgeRefined;
			bool blobResegmented, maximaHints, peripheralCenters, resegmentationMask;
			bool labels;
			std::vector<bool> showBlobsMaximaStages;
		} options;
		
		std::shared_ptr<BlobEmulationVis> vis;
		std::shared_ptr<BlobEmulationResults> result;
		std::shared_ptr<BlobEmulationResults> newResults;
	} emulation = {};

	struct {
		bool precalculated = false;
		std::vector<VisPoint> fullBorder, usedBorder;
		unsigned int undistortionTexID;
		Eigen::Vector2f undistortMapScale;
	} calibration = {};

	struct ObservationVis
	{ // Exclusively updated externally
		BlockedVector<Eigen::Vector2f> ptsStable;
		std::vector<Eigen::Vector2f> ptsUnstable;
		std::vector<Eigen::Vector2f> ptsTemp;
		std::vector<Eigen::Vector2f> ptsInactive;
		std::vector<SceneLabel> labels;
	};
	SynchronisedS<ObservationVis> observations = {};

	struct {
		bool show;
		// A grid of average error values for VIS_ERROR_VIS
		bool hasMap;
		Eigen::Vector2i mapSize = Eigen::Vector2i::Zero();
		unsigned int errorMapVBO, outlierMapVBO, coverageMapVBO;
		// Individual observation points and their errors for PHASE_Calibration_Point
		unsigned int errorPtsVBO, errorPtsCnt;
	} errors = {};
};

struct CameraView
{
	std::shared_ptr<TrackingCameraState> camera;
	CameraVisState vis = {};
	int gridX, gridY; // Predetermined grid
	ImVec2 size;
	bool resized;
	// ImGui State
	std::string ImGuiTitle;
	bool isDetached;
	int detachedIndex = -1;
};

#endif // UI_H