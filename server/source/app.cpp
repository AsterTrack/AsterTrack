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

#include "app.hpp"
#include "server.hpp"

#include "ui/shared.hpp" // Signals

#include "imgui/imgui.h" // IM_COL32

#include "util/util.hpp" // dtUS

#if !defined(INTERFACE_LINKED) && !defined(_MSC_VER)
#include <dlfcn.h>
#define ALLOW_DYNAMIC_LINKING
#else
//#include "dlfcn/dlfcn.h"
#endif

#include <cstdio>
#include <filesystem>

#ifdef __unix__
#include <unistd.h>
#elif defined(_WIN32)
#include <securitybaseapi.h>
#endif

AppState AppInstance;
ServerState StateInstance = {};
static std::atomic<bool> quitApp = { false };
static std::atomic<bool> closedUI = { false };
static TimePoint_t lastUIUpdate;


/* Interface */

InterfaceThread_t InterfaceThread;
SignalShouldClose_t SignalInterfaceShouldClose;
SignalLogUpdate_t SignalLogUpdate;
SignalCameraRefresh_t SignalCameraRefresh;
SignalPipelineUpdate_t SignalPipelineUpdate;
SignalObservationReset_t SignalObservationReset;
SignalServerEvent_t SignalServerEvent;

#ifdef INTERFACE_LINKED
extern "C" {
	bool _InterfaceThread();
	void _SignalShouldClose();
	void _SignalLogUpdate();
	void _SignalCameraRefresh(CameraID id);
	void _SignalPipelineUpdate();
	void _SignalObservationReset(long firstFrame);
	void _SignalServerEvent(ServerEvents event);
}
static inline bool AssumeInterface()
{
	InterfaceThread = (InterfaceThread_t)&_InterfaceThread;
	SignalInterfaceShouldClose = (SignalShouldClose_t)&_SignalShouldClose;
	SignalLogUpdate = (SignalLogUpdate_t)&_SignalLogUpdate;
	SignalCameraRefresh = (SignalCameraRefresh_t)&_SignalCameraRefresh;
	SignalPipelineUpdate = (SignalPipelineUpdate_t)&_SignalPipelineUpdate;
	SignalObservationReset = (SignalObservationReset_t)&_SignalObservationReset;
	SignalServerEvent = (SignalServerEvent_t)&_SignalServerEvent;
	return true;
}
#else
static inline bool AssumeInterface()
{
	return false;
}
#endif
static inline void UnlinkInterface(void *uidl = nullptr)
{
	void (*empty)(void) = [](){};
	InterfaceThread = []() -> bool { return false; };
	SignalInterfaceShouldClose = [](){};
	SignalLogUpdate = [](){};
	SignalCameraRefresh = [](CameraID){};
	SignalPipelineUpdate = [](){};
	SignalObservationReset = [](long){};
	SignalServerEvent = [](ServerEvents){};
#ifdef ALLOW_DYNAMIC_LINKING
	if (uidl) dlclose(uidl);
#endif
}
static inline bool LinkInterface(void *uidl)
{
#ifdef ALLOW_DYNAMIC_LINKING
	InterfaceThread = (InterfaceThread_t)dlsym(uidl, "_InterfaceThread");
	SignalInterfaceShouldClose = (SignalShouldClose_t)dlsym(uidl, "_SignalShouldClose");
	SignalLogUpdate = (SignalLogUpdate_t)dlsym(uidl, "_SignalLogUpdate");
	SignalCameraRefresh = (SignalCameraRefresh_t)dlsym(uidl, "_SignalCameraRefresh");
	SignalPipelineUpdate = (SignalPipelineUpdate_t)dlsym(uidl, "_SignalPipelineUpdate");
	SignalObservationReset = (SignalObservationReset_t)dlsym(uidl, "_SignalObservationReset");
	SignalServerEvent = (SignalServerEvent_t)dlsym(uidl, "_SignalServerEvent");
	return InterfaceThread && SignalInterfaceShouldClose && SignalLogUpdate && SignalCameraRefresh && SignalPipelineUpdate && SignalObservationReset && SignalServerEvent;
#else
	return true;
#endif
}


/* Logging implementation */

LogLevel LogMaxLevelTable[LMaxCategory];
LogLevel LogFilterTable[LMaxCategory];
thread_local LogCategory CurrentLogCategory = LDefault;
thread_local LogLevel CurrentLogLevel = LDebug;
thread_local int CurrentLogContext = -1;

const char* LogCategoryIdentifiers[LMaxCategory];
const char* LogCategoryDescriptions[LMaxCategory];
const char* LogLevelIdentifiers[LMaxLevel];
uint32_t LogLevelHexColors[LMaxLevel];

static void initialise_logging_strings();

/* Main Server Loop */

int main (void)
{
	// Set interface pointers to NOP functions
	UnlinkInterface();
	
	{ // Setup logging
		initialise_logging_strings();
		// Basic log file rotation
		if (std::filesystem::exists("log_1.txt"))
			std::filesystem::rename("log_1.txt", "log_2.txt");
		if (std::filesystem::exists("log.txt"))
			std::filesystem::rename("log.txt", "log_1.txt");
		GetApp().logFile.open("log.txt");

		// Init runtime max log levels
		// NOTE: Compile-time LOG_MAX_LEVEL takes priority!
		for (int i = 0; i < LMaxCategory; i++)
			LogMaxLevelTable[i] = LOG_MAX_LEVEL_DEFAULT;

		atexit([](){
			GetApp().FlushLog();
		});
	}

#ifdef __unix__
	if (geteuid() == 0)
	{
		printf("WARNING! Running with superuser privileges is NOT recommended and may expose you to security vulnerabilities!\n"
			"If you have troubles with permissions make sure your udev rules are set up properly - check documentation/readme for details.\n");
		LOG(LDefault, LWarn, "Running with superuser privileges is NOT recommended and may expose you to security vulnerabilities!");
		LOG(LDefault, LWarn, "If you have troubles with permissions make sure your udev rules are set up properly - check documentation/readme for details.\n");
	}
#elif defined(_WIN32)
	BOOL fIsRunAsAdmin = FALSE;
	PSID pAdminSid = NULL;
	if (CreateWellKnownSid(WinBuiltinAdministratorsSid, NULL, &pAdminSid))
	{
		if (CheckTokenMembership(NULL, pAdminSid, &fIsRunAsAdmin) && fIsRunAsAdmin)
		{
			printf("WARNING! Running with administrator privileges is NOT recommended and may expose you to security vulnerabilities!\n");
			LOG(LDefault, LWarn, "Running with administrator privileges is NOT recommended and may expose you to security vulnerabilities!");
		}
		FreeSid(pAdminSid);
	}
#endif

	{ // Init AsterTrack server
		if (!ServerInit(StateInstance))
			return -1;
	}

	LOGC(LInfo, "=======================\n");

	// TODO: Setup tray icon, etc.

	while (!quitApp.load())
	{
		// TODO: Wait for request to open interface

		void *uidl = nullptr;
		if (!AssumeInterface())
		{ // Open interface library and link it
#ifdef ALLOW_DYNAMIC_LINKING
			uidl = dlopen("astertrack-interface.so", RTLD_NOW);
			if (!uidl || !LinkInterface(uidl))
			{ // Error during linking
				const char * reason = dlerror();
				printf("Failed to link interface library: %s", reason);
				LOG(LDefault, LError, "Failed to link interface library: %s", reason);
				StateInstance.errors.push(asprintf_s("Failed to link interface library: %s", reason));
				fflush(stdout);
				std::this_thread::sleep_for(std::chrono::milliseconds(2000));
				exit(-1); // TODO: notify user that UI failed to open
				// But keeping as a zombie process is even worse
			}
#else
			exit(-1);
#endif
		}

		// Start UI thread that will be running for the lifetime of the UI
		lastUIUpdate = sclock::now();
		std::thread uiThread(InterfaceThread);
		while (!closedUI.load() && dtUS(sclock::now(), lastUIUpdate) < 20000)
		{ // 2s
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			GetApp().FlushLog();
		}

		if (!closedUI.load())
		{ // -> UI Hanging, ensure it knows to close, and wait for it
			SignalInterfaceShouldClose();
		}
		uiThread.join();
		closedUI = false;

		// Unlink interface library again
		UnlinkInterface(uidl);
	}

	// Exit manually before rest
	// This is because some statics initialised after server might be required while some threads are still exiting
	// Notably: vrpn_ConnectionManager, which would automatically delete all vrpn_Connection that might still be in use
	ServerExit(StateInstance);

	// If UI signaled to close the WHOLE app, just exit
	exit(0);
}

void AppState::SignalQuitApp()
{
	// Signal main thread to quit
	quitApp = true;
	SignalInterfaceShouldClose();
}

void AppState::SignalInterfaceClosed()
{
	// Signal main thread that UI was closed
	closedUI = true;
}

void AppState::FlushLog()
{
	std::unique_lock lock(logIteratorAccess);
	auto logs = logEntries.getView();
	auto flushStart = logs.pos(std::max(lastFlushed, logs.beginIndex()));
	for (auto entry = flushStart; entry != logs.end(); entry++)
	{ // Append to log file
		logFile << LogCategoryIdentifiers[entry->category] << " " << LogLevelIdentifiers[entry->level] << " : " << entry->log;
		if (entry->log.empty() || entry->log.back() != '\n')
			logFile << "\n";
	}
	logFile << std::flush;
	lastFlushed = logs.endIndex();
}


/* Logging implementation */

static void initialise_logging_strings()
{
	const char* defaultCategoryName = "!CAT";
	const char* defaultCategoryDesc = "!Missing Category Description";
	for (int i = 0; i < LMaxCategory; i++)
	{
		LogCategoryIdentifiers[i] = defaultCategoryName;
		LogCategoryDescriptions[i] = defaultCategoryDesc;
	}

	LogCategoryIdentifiers[LDefault] 				= "Std ";
	LogCategoryIdentifiers[LGUI] 					= "GUI ";
	LogCategoryIdentifiers[LPipeline] 				= "Pipe";
	LogCategoryIdentifiers[LSimulation] 			= "Sim ";
	LogCategoryIdentifiers[LSequence] 				= "Seq ";
	LogCategoryIdentifiers[LOptimisation]			= "Opt ";
	LogCategoryIdentifiers[LPointCalib]				= "cCal";
	LogCategoryIdentifiers[LPointReconstruction]	= "cRec";
	LogCategoryIdentifiers[LTargetCalib]			= "tCal";
	LogCategoryIdentifiers[LTargetReconstruction]	= "tRec";
	LogCategoryIdentifiers[LTracking]				= "Trk ";
	LogCategoryIdentifiers[LCluster]				= "Clst";
	LogCategoryIdentifiers[LTriangulation]			= "Tri ";
	LogCategoryIdentifiers[LDetection2D]			= "tD2D";
	LogCategoryIdentifiers[LDetection3D]			= "tD3D";
	LogCategoryIdentifiers[LTrackingMatch]			= "tM2D";
	LogCategoryIdentifiers[LTrackingOpt]			= "tOpt";
	LogCategoryIdentifiers[LTrackingFilter]			= "tFlt";
	LogCategoryIdentifiers[LTrackingIMU]			= "tIMU";

	LogCategoryIdentifiers[LStreaming]				= "Strm";
	LogCategoryIdentifiers[LTimesync]				= "Sync";
	LogCategoryIdentifiers[LSOF]					= "SoF ";
	LogCategoryIdentifiers[LParsing]				= "Pars";
	LogCategoryIdentifiers[LServer]					= "Serv";
	LogCategoryIdentifiers[LUSB]					= "USB ";
	LogCategoryIdentifiers[LProtocol]				= "Prot";
	LogCategoryIdentifiers[LControllerDevice]		= "Ctrl";
	LogCategoryIdentifiers[LCameraDevice]			= "Cam ";
	LogCategoryIdentifiers[LCameraEmulation]		= "Emul";
	LogCategoryIdentifiers[LFirmwareUpdate]			= "FwUp";
	LogCategoryIdentifiers[LIMUDriver]				= "dIMU";
	LogCategoryIdentifiers[LIO]						= "I/O ";

	LogCategoryDescriptions[LDefault] 				= "Default";
	LogCategoryDescriptions[LGUI] 					= "GUI / Visualisation";
	LogCategoryDescriptions[LPipeline] 				= "Pipeline and Algorithm";
	LogCategoryDescriptions[LSimulation] 			= "Simulation";
	LogCategoryDescriptions[LSequence] 				= "2D Sequences";
	LogCategoryDescriptions[LOptimisation]			= "Std. Optimisation";
	LogCategoryDescriptions[LPointCalib]			= "Camera Calibration";
	LogCategoryDescriptions[LPointReconstruction]	= "Camera Reconstruction";
	LogCategoryDescriptions[LTargetCalib]			= "Target Calibration";
	LogCategoryDescriptions[LTargetReconstruction]	= "Target Reconstruction";
	LogCategoryDescriptions[LTracking]				= "Tracking Pipeline";
	LogCategoryDescriptions[LCluster]				= "Clustering";
	LogCategoryDescriptions[LTriangulation]			= "Triangulation";
	LogCategoryDescriptions[LDetection2D]			= "2D Detection";
	LogCategoryDescriptions[LDetection3D]			= "3D Detection";
	LogCategoryDescriptions[LTrackingMatch]			= "Target Matching";
	LogCategoryDescriptions[LTrackingOpt]			= "Target Optimisation";
	LogCategoryDescriptions[LTrackingFilter]		= "Filtering";
	LogCategoryDescriptions[LTrackingIMU]			= "IMU Tracking";

	LogCategoryDescriptions[LStreaming]				= "Streaming";
	LogCategoryDescriptions[LTimesync]				= "TimeSync";
	LogCategoryDescriptions[LSOF]					= "Start of Frame";
	LogCategoryDescriptions[LParsing]				= "Parsing";
	LogCategoryDescriptions[LServer]				= "Server";
	LogCategoryDescriptions[LUSB]					= "USB Transfers";
	LogCategoryDescriptions[LProtocol]				= "Block Protocol";
	LogCategoryDescriptions[LControllerDevice]		= "Controller Logs";
	LogCategoryDescriptions[LCameraDevice]			= "Camera Logs";
	LogCategoryDescriptions[LCameraEmulation]		= "Emulated Blob Detection";
	LogCategoryDescriptions[LFirmwareUpdate]		= "Firmware Update";
	LogCategoryDescriptions[LIMUDriver]				= "IMU Driver";
	LogCategoryDescriptions[LIO]					= "Integrations";

	LogLevelIdentifiers[LTrace]  = "TRACE";
	LogLevelIdentifiers[LDebug]  = "DEBUG";
	LogLevelIdentifiers[LDarn]   = "DWARN";
	LogLevelIdentifiers[LInfo]   = "INFO ";
	LogLevelIdentifiers[LWarn]   = "WARN ";
	LogLevelIdentifiers[LError]  = "ERROR";
	LogLevelIdentifiers[LOutput] = "OUT  ";

	LogLevelHexColors[LTrace]  = IM_COL32(0xAA, 0xAA, 0xAA, 0xAA);
	LogLevelHexColors[LDebug]  = IM_COL32(0xCC, 0xCC, 0xCC, 0xFF);
	LogLevelHexColors[LDarn]   = IM_COL32(0xBB, 0x66, 0x33, 0xFF);
	LogLevelHexColors[LInfo]   = IM_COL32(0xFF, 0xFF, 0xFF, 0xFF);
	LogLevelHexColors[LWarn]   = IM_COL32(0xDD, 0x88, 0x44, 0xFF);
	LogLevelHexColors[LError]  = IM_COL32(0xEE, 0x44, 0x44, 0xFF);
	LogLevelHexColors[LOutput] = IM_COL32(0xBB, 0xBB, 0x44, 0xFF);
}

// TODO: switch to stb_sprintf, supposedly faster
#define FORMAT_TO_STRING(FORMAT, STRING) \
	va_list argp; \
	va_start(argp, FORMAT); \
	int size = std::vsnprintf(nullptr, 0, FORMAT, argp); \
	va_end(argp); \
	if (size <= 0) return -1; \
	STRING.resize(size); \
	va_start(argp, FORMAT); \
	std::vsnprintf(STRING.data(), size+1, FORMAT, argp); \
	va_end(argp); \

int PrintLog(LogCategory category, LogLevel level, int context, const char *format, ...)
{
	AppState::LogEntry entry = {};
	entry.category = category;
	entry.level = level;
	entry.context = context;
	FORMAT_TO_STRING(format, entry.log);

	GetApp().logEntries.push_back(std::move(entry));

	if (LogFilterTable[category] <= level)
		SignalLogUpdate();

	return size;
}

int PrintLogCont(LogCategory category, LogLevel level, int context, const char *format, ...)
{
	AppState::LogEntry entry = {};
	entry.category = category;
	entry.level = level;
	entry.context = context;
	FORMAT_TO_STRING(format, entry.log);

	bool foundCont = false;
	{
		std::unique_lock lock(GetApp().logIteratorAccess);
		auto logs = GetApp().logEntries.getView<false>();
		auto cont = logs.end();
		int search = 50;
		while (search-- > 0 && cont-- != logs.begin())
		{ // Append to last continued log entry
			if (cont->category == category && cont->context == context)
			{
				cont->log.insert(cont->log.end(), entry.log.begin(), entry.log.end());
				foundCont = true;
				break;
			}
		}
	}

	if (!foundCont)
	{ // Start new continued log entry
		GetApp().logEntries.push_back(std::move(entry));
	}

	if (LogFilterTable[category] <= level)
		SignalLogUpdate();

	return size;
}