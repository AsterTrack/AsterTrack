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
#include <algorithm>

#include "error.hpp"

/*
 * Utilities (logging, statistics, etc)
 */


/* Time */

typedef std::chrono::steady_clock sclock;
typedef sclock::time_point TimePoint_t;
typedef std::chrono::high_resolution_clock pclock;
typedef int64_t dt_t;
template<typename TimePoint>
static inline dt_t dtUS(TimePoint t0, TimePoint t1)
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

template<typename DstTimePointT, typename SrcTimePointT,
	typename SrcDurationT = typename SrcTimePointT::duration,
	typename DstDurationT = typename DstTimePointT::duration,
	typename DstClockT = typename DstTimePointT::clock,
	typename SrcClockT = typename SrcTimePointT::clock>
static inline std::pair<DstTimePointT,SrcTimePointT> getAccurateClockReference(
	SrcDurationT &tolerance,
	int &limit)
{ // Based on https://stackoverflow.com/a/35293183
	assert(limit > 0);
	auto itercnt = 0;
	SrcTimePointT src_now;
	DstTimePointT dst_now;
	SrcDurationT epsilon(std::numeric_limits<typename SrcDurationT::rep>::max());
	do
	{
		const auto src_before = SrcClockT::now();
		dst_now = DstClockT::now();
		const auto src_after = SrcClockT::now();
		const auto src_diff = src_after - src_before;
		if (src_diff.count() > 0 && src_diff < epsilon)
		{
			src_now = src_before + src_diff / 2;
			epsilon = src_diff;
		}
		if (++itercnt >= limit)
			break;
	}
	while (epsilon > tolerance);
	tolerance = epsilon;
	limit = itercnt;
	return { dst_now, src_now };
}

template<typename DstTimePointT, typename SrcTimePointT>
static std::pair<DstTimePointT,SrcTimePointT> getAccurateClockReference()
{
	typename SrcTimePointT::duration tolerance = std::chrono::nanoseconds(200);
	int limit = 5;
	return getAccurateClockReference<DstTimePointT, SrcTimePointT>(tolerance, limit);
}

template<typename DstTimePointT, typename SrcTimePointT>
static inline DstTimePointT convertClockWithRef(SrcTimePointT t, const std::pair<DstTimePointT, SrcTimePointT> &ref)
{
	return ref.first + std::chrono::duration_cast<typename DstTimePointT::duration>(t - ref.second);
}

template<typename DstTimePointT, typename SrcTimePointT>
static DstTimePointT convertClockAccurate(const SrcTimePointT t,
	typename SrcTimePointT::duration tolerance = std::chrono::nanoseconds(200),
	int limit = 5)
{
	auto ref_now = getAccurateClockReference<DstTimePointT, SrcTimePointT>(tolerance, limit);
	return convertClockWithRef(t, ref_now);
}

template<typename DstTimePointT, typename SrcTimePointT>
static inline DstTimePointT convertClock(SrcTimePointT t)
{
	const auto src_now = SrcTimePointT::clock::now();
	const auto dst_now = DstTimePointT::clock::now();
	return convertClockWithRef<DstTimePointT, SrcTimePointT>(t, { dst_now, src_now });
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

template<typename DIFF = int64_t, typename UNSIGNED = uint64_t>
static inline DIFF diffUnsigned(UNSIGNED a, UNSIGNED b)
{
	return b < a? -(DIFF)(a - b) : (DIFF)(b - a);
}

template<typename TRUNC, typename DIFF>
static inline DIFF shortDiff(TRUNC a, TRUNC b, DIFF bias, TRUNC max = std::numeric_limits<TRUNC>::max())
{
	static_assert(std::numeric_limits<TRUNC>::lowest() == 0);
	assert(a <= max && b <= max);
	DIFF passed = diffUnsigned<DIFF, TRUNC>(a, b);
	if (passed < -bias)
		passed = passed + 1 + max;
	else if (passed > max-bias)
		passed = passed - 1 - max;
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


/* More performant remove when relative order does not matter */

template<class ForwardIt, class T = typename std::iterator_traits<ForwardIt>::value_type>
ForwardIt remove_swap(ForwardIt first, ForwardIt last, const T& value)
{
    first = std::find(first, last, value);
    if (first == last) return first;
	std::advance(last, -1);
	if (first != last)
		std::swap(*first, *last);
	return last;
}

template<class ForwardIt, class UnaryPred>
ForwardIt remove_swap_if(ForwardIt first, ForwardIt last, UnaryPred p)
{
    first = std::find_if(first, last, p);
    if (first == last) return first;
	std::advance(last, -1);
	if (first != last)
		std::swap(*first, *last);
	return last;
}

#endif // UTIL_H