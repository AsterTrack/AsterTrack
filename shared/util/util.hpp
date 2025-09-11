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

#ifndef UTIL_H
#define UTIL_H

// Just windows things
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <chrono>

#include <sstream>
#include <iomanip>
#include <string>
#include <cstdio>
#include <cassert>
#include <cstdarg>

/*
 * Utilities (logging, statistics, etc)
 */


/* Time */

typedef std::chrono::steady_clock sclock;
typedef sclock::time_point TimePoint_t;
typedef std::chrono::high_resolution_clock pclock;
template<typename TimePoint>
static inline long dtUS(TimePoint t0, TimePoint t1)
{
	return std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
}
template<typename Scalar = float, typename TimePoint>
static inline Scalar dtMS(TimePoint t0, TimePoint t1)
{
	return dtUS(t0, t1) / (Scalar)1000.0;
}
template<typename Scalar = float, typename TimePoint>
static inline Scalar dtS(TimePoint t0, TimePoint t1)
{
	return dtUS(t0, t1) / (Scalar)1000000.0;
}


/* asprintf_s */

#ifdef _MSC_VER

// Might not work in the community edition of VS, only in the enterprise version, idk and idc
#ifdef _USE_ATTRIBUTES_FOR_SAL
#undef _USE_ATTRIBUTES_FOR_SAL
#define _USE_ATTRIBUTES_FOR_SAL 1
#endif
#include <sal.h>

static std::string asprintf_s( _Printf_format_string_ const char *format, ...);

#else

static std::string asprintf_s( const char *format, ...) __attribute__ ((format (printf,1,2)));

#endif

static std::string asprintf_s( const char *format, ...)
{
	va_list argp;
	va_start(argp, format);
	int size = std::vsnprintf(nullptr, 0, format, argp);
	va_end(argp);
	if (size <= 0)
		return "[Error during formatting]";
	std::string buffer;
	buffer.resize(size); // Since C++11 providing a null-terminated string internally
	va_start(argp, format);
	std::vsnprintf(buffer.data(), size+1, format, argp);
	va_end(argp);
	return buffer;
}


/* shortDiff */

template<typename UINT, typename INT>
static inline INT shortDiff(UINT a, UINT b, INT bias, UINT overflow)
{
	static_assert(std::numeric_limits<UINT>::lowest() == 0);
	assert(a < overflow && b < overflow);
	INT passed = ((INT)b) - a;
	if (passed < -bias)
		passed += overflow;
	else if (passed > overflow-bias)
		passed -= overflow;
	return passed;
}


/* printXXX */

#ifdef EIGEN_DEF_H // Not the best solution, requires correct ordering of includes, but works
/**
 * Stringify a matrix over multiple lines
 */
template<typename Derived>
static inline std::string printMatrix(const Eigen::MatrixBase<Derived> &mat)
{
	std::stringstream ss;
	ss << std::setprecision(3) << std::fixed << mat;
	return ss.str();
}
#endif

inline void printBuffer(std::stringstream &ss, uint8_t *buffer, int size)
{
	ss << "0x" << std::uppercase << std::hex;
	for (int i = 0; i < size; i++)
		ss << std::setfill('0') << std::setw(2) << (int)buffer[i];
}

#endif // UTIL_H