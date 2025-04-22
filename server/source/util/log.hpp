/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#if defined(LOG_H) && defined(LOG_MAX_LEVEL) && (!defined(FIXED_LOG_LEVEL) || FIXED_LOG_LEVEL != LOG_MAX_LEVEL)
#error Conflicting LOG_MAX_LEVEL overrides, make sure LOG_MAX_LEVEL is defined before the first log.hpp inclusion in the file!
#endif

#ifndef LOG_MAX_LEVEL_DEFAULT
#define LOG_MAX_LEVEL_DEFAULT LDebug
#endif

#ifndef LOG_H
#define LOG_H

#include <cstdint>

enum LogCategory {
	LDefault,
	LGUI,
	// Algorithms
	LPipeline,
		LSimulation,
		LSequence,
		LOptimisation,
		LPointCalib,
		LPointReconstruction,
		LTargetCalib,
		LTargetReconstruction,
		LTracking,
			LTriangulation,
			LDetection2D,
			LDetection3D,
			LTrackingMatch,
			LTrackingOpt,
			LTrackingFilter,
	// Realtime communications, mostly for debugging:
	LStreaming,
		LTimesync,
		LSOF,
		LParsing,
	LServer,
	LUSB,
		LProtocol,
	// Logs from devices (or strongly related and of no general interest):
	LControllerDevice,
	LCameraDevice,
	LCameraEmulation,
	LIO,
	LMaxCategory
};
enum LogLevel : char {
	LTrace,
	LDebug,
	LDarn, // Debug Warn
	LInfo,
	LWarn,
	LError,
	LOutput,
	LMaxLevel
};

#ifdef _MSC_VER

// Might not work in the community edition of VS, only in the enterprise version, idk and idc
#ifdef _USE_ATTRIBUTES_FOR_SAL
#undef _USE_ATTRIBUTES_FOR_SAL
#define _USE_ATTRIBUTES_FOR_SAL 1
#endif
#include <sal.h>

int PrintLog(LogCategory c, LogLevel l, int ctx, _Printf_format_string_ const char *f, ...);
int PrintLogCont(LogCategory c, LogLevel l, int ctx, _Printf_format_string_ const char *f, ...);

#else

int PrintLog(LogCategory c, LogLevel l, int ctx, const char *f, ...) __attribute__ ((format (printf,4,5)));
int PrintLogCont(LogCategory c, LogLevel l, int ctx, const char *f, ...) __attribute__ ((format (printf,4,5)));

#endif


extern LogLevel LogMaxLevelTable[LMaxCategory];
extern LogLevel LogFilterTable[LMaxCategory];
extern thread_local LogCategory CurrentLogCategory;
extern thread_local LogLevel CurrentLogLevel;
extern thread_local int CurrentLogContext;

extern const char* LogCategoryIdentifiers[LMaxCategory];
extern const char* LogCategoryDescriptions[LMaxCategory];
extern const char* LogLevelIdentifiers[LMaxLevel];
extern uint32_t LogLevelHexColors[LMaxLevel];

#ifdef LOG_MAX_LEVEL
#define SHOULD_LOG(CATEGORY, LEVEL) ((LEVEL) >= (LOG_MAX_LEVEL))
#define FIXED_LOG_LEVEL LOG_MAX_LEVEL
#else
#define SHOULD_LOG(CATEGORY, LEVEL) ((LEVEL) >= LOG_MAX_LEVEL_DEFAULT && (LEVEL) >= LogMaxLevelTable[CATEGORY])
#endif
#define SHOULD_LOGC(LEVEL) SHOULD_LOG(CurrentLogCategory, LEVEL)
#define SHOULD_LOGCL() SHOULD_LOG(CurrentLogCategory, CurrentLogLevel)

#define LOG(CATEGORY, LEVEL, ...) { \
	if (SHOULD_LOG(CATEGORY, LEVEL)) \
		PrintLog(CATEGORY, LEVEL, CurrentLogContext, __VA_ARGS__); \
}

#define LOGC(LEVEL, ...) { \
	if (SHOULD_LOG(CurrentLogCategory, LEVEL)) \
		PrintLog(CurrentLogCategory, LEVEL, CurrentLogContext, __VA_ARGS__); \
}

#define LOGL(CATEGORY, ...) { \
	if (SHOULD_LOG(CATEGORY, CurrentLogLevel)) \
		PrintLog(CATEGORY, CurrentLogLevel, CurrentLogContext, __VA_ARGS__); \
}

#define LOGCL(...) { \
	if (SHOULD_LOG(CurrentLogCategory, CurrentLogLevel)) \
		PrintLog(CurrentLogCategory, CurrentLogLevel, CurrentLogContext, __VA_ARGS__); \
}

#define LOGCONTC(LEVEL, ...) { \
	if (SHOULD_LOG(CurrentLogCategory, LEVEL)) \
		PrintLog(CurrentLogCategory, LEVEL, CurrentLogContext, __VA_ARGS__); \
}

#define LOGCONT(CATEGORY, LEVEL, ...) { \
	if (SHOULD_LOG(CATEGORY, LEVEL)) \
		PrintLogCont(CATEGORY, LEVEL, CurrentLogContext, __VA_ARGS__); \
}


/* #define LOG(CATEGORY, LEVEL, ...) do { \
	if (SHOULD_LOG(CATEGORY, LEVEL)) \
		PrintLog(CATEGORY, LEVEL, CurrentLogContext, __VA_ARGS__); \
} while(false)

#define LOGC(LEVEL, ...) do { \
	if (SHOULD_LOG(CurrentLogCategory, LEVEL)) \
		PrintLog(CurrentLogCategory, LEVEL, CurrentLogContext, __VA_ARGS__); \
} while(false)

#define LOGL(CATEGORY, ...) do { \
	if (SHOULD_LOG(CATEGORY, CurrentLogLevel)) \
		PrintLog(CATEGORY, CurrentLogLevel, CurrentLogContext, __VA_ARGS__); \
} while(false)

#define LOGCL(...) do { \
	if (SHOULD_LOG(CurrentLogCategory, CurrentLogLevel)) \
		PrintLog(CurrentLogCategory, CurrentLogLevel, CurrentLogContext, __VA_ARGS__); \
} while(false)

#define LOGCONTC(LEVEL, ...) do { \
	if (SHOULD_LOG(CurrentLogCategory, LEVEL)) \
		PrintLog(CurrentLogCategory, LEVEL, CurrentLogContext, __VA_ARGS__); \
} while(false)

#define LOGCONT(CATEGORY, LEVEL, ...) do { \
	if (SHOULD_LOG(CATEGORY, LEVEL)) \
		PrintLogCont(CATEGORY, LEVEL, CurrentLogContext, __VA_ARGS__); \
} while(false) */

struct ScopedLogCategory
{
	LogCategory prev;
	ScopedLogCategory(LogCategory category, bool force = true) { 
		prev = CurrentLogCategory;
		if (force || prev == LDefault)
			CurrentLogCategory = category;
	}
	~ScopedLogCategory() { CurrentLogCategory = prev; }
};

struct ScopedLogLevel
{
	LogLevel prev;
	ScopedLogLevel(LogLevel level) { 
		prev = CurrentLogLevel;
		CurrentLogLevel = level;
	}
	~ScopedLogLevel() { CurrentLogLevel = prev; }
};

struct ScopedLogContext
{
	int prev;
	ScopedLogContext(int context) { 
		prev = CurrentLogContext;
		CurrentLogContext = context;
	}
	~ScopedLogContext() { CurrentLogContext = prev; }
};

#endif // LOG_H