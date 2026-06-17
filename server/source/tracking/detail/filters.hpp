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

#ifndef FILTERS_H
#define FILTERS_H

#include "util/util.hpp" // TimePoint_t
#include "Eigen/Core" // Eigen::MatrixBase, Eigen::QuaternionBase
#include "flexkalman/EigenQuatExponentialMap.h" // quat_exp, quat_ln

namespace FilterSpecialisation
{
	template<typename T, typename = std::enable_if_t<std::is_scalar_v<T>>>
	static T lerp(const T &a, const T &b, T factor) { return factor * b + (1.0 - factor) * a; }
	template<typename D>
	static D lerp(const Eigen::MatrixBase<D> &a, const Eigen::MatrixBase<D> &b, typename D::Scalar factor) { return factor * b + (1.0 - factor) * a; }
	template<typename D>
	static D lerp(const Eigen::QuaternionBase<D> &a, const Eigen::QuaternionBase<D> &b, typename D::Scalar factor) { return a.slerp(factor, b); }

	template<typename T, typename = std::enable_if_t<std::is_scalar_v<T>>>
	static T delta(const T &a, const T &b) { return b - a; }
	template<typename D>
	static D delta(const Eigen::MatrixBase<D> &a, const Eigen::MatrixBase<D> &b) { return b - a; }
	template<typename D>
	static Eigen::Vector3<typename D::Scalar> delta(const Eigen::QuaternionBase<D> &a, const Eigen::QuaternionBase<D> &b) { return flexkalman::util::quat_ln(b * a.conjugate()); }

	template<typename T, typename = std::enable_if_t<std::is_scalar_v<T>>>
	static T norm(const T &a) { return a; }
	template<typename D>
	static D::Scalar norm(const Eigen::MatrixBase<D> &a) { return a.norm(); }
	template<typename D>
	static D::Scalar norm(const Eigen::QuaternionBase<D> &a) { return a.norm(); }
};

template<typename T = float, typename Scalar = T>
struct LowPassFilter
{
	bool initialised;
	T filtered;
	Scalar alpha;

	inline LowPassFilter(Scalar alpha) : initialised(false), alpha(alpha) {}

	inline T filter(T sample)
	{
		if (initialised)
		{
			filtered = FilterSpecialisation::lerp(filtered, sample, alpha);
		}
		else
		{
			filtered = sample;
			initialised = true;
		}
		return filtered;
	}
};

template<typename T = float, typename Scalar = T, typename Delta = T>
struct OneEuroFilter
{
	TimePoint_t filterTime;
	Scalar cutoffBase, cutoffBeta, cutoffDelta;
	LowPassFilter<T,Scalar> filterValue;
	LowPassFilter<Delta,Scalar> filterDelta;

	inline OneEuroFilter(Scalar cutoffBase = 1, Scalar cutoffBeta = 0, Scalar cutoffDelta = 1) :
		cutoffBase(cutoffBase), cutoffBeta(cutoffBeta), cutoffDelta(cutoffDelta),
		filterValue(1), filterDelta(1) {}

	inline void setParams(Scalar cutoffBase = 1, Scalar cutoffBeta = 0, Scalar cutoffDelta = 1)
	{
		this->cutoffBase = cutoffBase;
		this->cutoffBeta = cutoffBeta;
		this->cutoffDelta = cutoffDelta;
	}

	inline Scalar alpha(Scalar cutoff, Scalar frequency)
	{
		Scalar tau = 1.0 / (2 * M_PI * cutoff);
		return 1.0 / (1.0 + tau * frequency);
	}

	inline T filter(T sample, TimePoint_t time)
	{
		if (!filterValue.initialised)
		{ // Initialise: No frequency, no value to filter
			filterTime = time;
			return filterValue.filter(sample);
		}

		// Get observed frequency
		Scalar frequency = 1.0 / dtS(filterTime, time);
		filterTime = time;

		// Estimate velocity from filtered delta
		Delta delta = FilterSpecialisation::delta(filterValue.filtered, sample) * frequency;
		filterDelta.alpha = alpha(cutoffDelta, frequency);
		Delta velocity = filterDelta.filter(delta);
		// Calculate speed from estimate velocity
		Scalar speed = FilterSpecialisation::norm(velocity);

		// Filter final value with cutoff based on speed
		Scalar cutoff = cutoffBase + cutoffBeta * speed;
		filterValue.alpha = alpha(cutoff, frequency);
		return filterValue.filter(sample);
	}

	inline void reset()
	{
		filterValue.initialised = false;
		filterDelta.initialised = false;
	}
};

#endif // FILTERS_H