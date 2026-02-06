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

/* Dummy file to disable additional logging for server in shared code */

#ifndef LOG_H
#define LOG_H

#include <cstdio>
#include <cinttypes> // For cross-platform 64bit labels (ll(u) on windows, l(u) on linux)

#ifndef LOG_MAX_LEVEL
#ifdef NDEBUG
#define LOG_MAX_LEVEL LInfo
#else
#define LOG_MAX_LEVEL LDarn
#endif
#endif

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

#define STR_CAT(CAT) #CAT
#define LOG_REPLACE(CAT, LVL, STR, ...) { if (LVL >= LOG_MAX_LEVEL) printf(STR_CAT(CAT) ": " STR "\n" __VA_OPT__(,) __VA_ARGS__); }

#define LOG(CATEGORY, LEVEL, ...) LOG_REPLACE(CATEGORY, LEVEL, __VA_ARGS__)
#define LOGC(LEVEL, ...) LOG_REPLACE(L, LEVEL, __VA_ARGS__)
#define LOGL(CATEGORY, ...) LOG_REPLACE(CATEGORY, LDebug, __VA_ARGS__)
#define LOGCL(...) LOG_REPLACE(L, LDebug, __VA_ARGS__)
#define LOGCONTC(LEVEL, ...) LOG_REPLACE(L, LEVEL, __VA_ARGS__)
#define LOGCONT(CATEGORY, LEVEL, ...) LOG_REPLACE(CATEGORY, LEVEL, __VA_ARGS__)

#endif // LOG_H