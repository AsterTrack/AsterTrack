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

#ifndef STATS_H
#define STATS_H

#ifndef _USE_MATH_DEFINES // For MSVC
#define _USE_MATH_DEFINES // Not enough if cmath was included previously, so defined in build system
#endif
#include <cmath>
#include <vector>
#include <array>

enum StatOptions
{
	StatFloating = 1,
	StatExtremas = 2,
	StatDistribution = 4,
};

template<typename Scalar = float, int Options = 0>
struct StatValue
{
	Scalar sum, avg, floating, M2, min, max;
	int num, floatCount;

	StatValue() { reset(); };
	StatValue(int floatingCount) : floatCount(floatingCount) { reset(); }

	/**
	 * Updates the given statistical value with a new value
	 */
	inline void update(Scalar val)
	{
		if (num == 0)
		{
			avg = val; // For M2 calculation
			if constexpr ((Options & StatExtremas) == StatExtremas)
			{
				min = val;
				max = val;
			}
			if constexpr ((Options & StatFloating) == StatFloating)
				floating = val;
		}
		else if constexpr ((Options & StatExtremas) == StatExtremas)
		{
			if (min > val) min = val;
			if (max < val) max = val;
		}
		// Welford algorithm
		num++;
		sum += val;
		Scalar fac = 1.0/num;
		Scalar diff = val - avg;
		avg += diff * fac;
		if constexpr ((Options & StatDistribution) == StatDistribution)
		{
			Scalar diff2 = val - avg;
			M2 += diff * diff2;
		}
		// Floating average
		if constexpr ((Options & StatFloating) == StatFloating)
		{
			int floatNum = num < floatCount? num : floatCount;
			floating = (floating*floatNum + val)/(floatNum+1);
		}
	}

	inline Scalar variance() const { return num < 2? 0.0f : M2/(num-1); }
	inline Scalar stdDev() const { return std::sqrt(variance()); }

	template<int O>
	inline void merge(StatValue<Scalar, O> &o)
	{
		if (o.num == 0) return;
		if (num == 0)
		{
			*this = o;
			return;
		}
		int total = num + o.num;
		float fac = 1.0f/total;
		float cAvg = (avg*num + o.avg*o.num) * fac;
		if constexpr ((Options & StatDistribution) == StatDistribution && (O & StatDistribution) == StatDistribution)
		{
			float dAvg1 = (avg-cAvg), dAvg2 = (o.avg-cAvg);
			M2 = M2 + dAvg1*dAvg1*num + o.M2 + dAvg2*dAvg2*o.num;
		}
		num = total;
		sum += o.sum;
		avg = cAvg;
		if constexpr ((Options & StatExtremas) == StatExtremas && (O & StatExtremas) == StatExtremas)
		{
			if (min > o.min) min = o.min;
			if (max < o.max) max = o.max;
		}
		if constexpr ((Options & StatFloating) == StatFloating && (O & StatFloating) == StatFloating)
		{ // Approximate
			int floatNum1 = num < floatCount? num : floatCount;
			int floatNum2 = o.num < o.floatCount? o.num : o.floatCount;
			floating = (floating*floatNum1 + o.floating*floatNum2)/(floatNum1+floatNum2);
		}
	}

	/**
	 * Resets the statistical value to the default state
	 */
	inline void reset()
	{
	    avg = sum = floating = M2 = min = max = (Scalar)0.0;
		num = 0;
	}

	inline float probabilityAbove(float value)
	{
		return 0.5f * erfc((value-avg)/stdDev() * M_SQRT1_2);
	}

	inline float probabilityBelow(float value)
	{
		return 0.5f + 0.5f * erf((value-avg)/stdDev() * M_SQRT1_2);
	}

	inline float probabilityDeviationWithin(float value)
	{
		return erf(value/stdDev());
	}
};

typedef StatValue<float,StatFloating|StatExtremas|StatDistribution> Statisticsf;
typedef StatValue<float,StatExtremas|StatDistribution> StatBlockf;
typedef StatValue<float,StatDistribution> StatDistf;
typedef StatValue<float,StatFloating> StatFloatingf;
typedef StatValue<float> StatAvgf;

/**
 * Keeps min/max values where the following extrema are relevant
 */
 template<typename Scalar, int N = 2>
struct MultipleExtremum
{
	using Dynamic = std::conditional_t<N <= 0, std::true_type, std::false_type>;
	using Container = std::conditional_t<Dynamic::value, std::vector<Scalar>, std::array<Scalar, (std::size_t)N>>;
	Container rank;
	Scalar signal;
	MultipleExtremum(Scalar init) : signal(init)
	{
		if constexpr (!Dynamic::value)
		 	std::fill_n(rank.begin(), N, init);
	}

	void max(Scalar value, int maxMatches = -1)
	{
		if (rank.size() >= std::max(1,maxMatches) && rank.back() != signal && rank.back() > value) return;
		int i = 0;
		for (; i < rank.size(); i++)
			if (rank[i] == signal || rank[i] < value) break;
		for (; i < rank.size(); i++)
			std::swap(value, rank[i]);
		if constexpr (Dynamic::value)
		{
			rank.reserve(maxMatches);
			if (rank.size() < maxMatches)
				rank.push_back(value);
		}
	}

	void min(Scalar value, int maxMatches = -1)
	{
		if (rank.size() >= std::max(1,maxMatches) && rank.back() != signal && rank.back() < value) return;
		int i = 0;
		for (; i < N; i++)
			if (rank[i] == signal || rank[i] > value) break;
		for (; i < N; i++)
			std::swap(value, rank[i]);
		if constexpr (Dynamic::value)
		{
			rank.reserve(maxMatches);
			if (rank.size() < maxMatches)
				rank.push_back(value);
		}
	}

	int weakest() const
	{
		for (int i = 0; i < rank.size(); i++)
			if (rank[i] == signal)
				return i-1;
		return N-1;
	}

	Scalar average() const
	{
		if (rank.empty()) return signal;
		Scalar avg = 0.0;
		for (int i = 0; i < rank.size(); i++)
		{
			if constexpr (!Dynamic::value)
				if (rank[i] == signal)
					return i == 0? signal : avg/i;
			avg += rank[i];
		}
		return avg / rank.size();
	}

	bool hasAll() const
	{
		if constexpr (Dynamic::value)
			return rank.size() == rank.capacity();
		else
			return rank[N-1] != signal;
	}
	bool hasAny() const
	{
		if constexpr (Dynamic::value)
			return !rank.empty();
		else
			return rank[0] != signal;
	}
};

/**
 * Keeps min/max values where the second extremum is relevant
 */
template<typename Scalar>
using DualExtremum = MultipleExtremum<Scalar, 2>;


/**
 * Raise x to the power of integer n.
 * Very quick for small n.
 */
template<typename T>
static constexpr inline T pown(T x, unsigned n)
{
	T result = 1;
	while (n)
	{
		if (n&1) result *= x;
		x *= x;
		n >>= 1;
	}
	return result;
}

/**
 * Keeps a value from changing excessively within a given range
 */
template<typename Scalar, int Precision = 2, float Factor = 1.5f>
struct Hysteresis
{
	float raw, rounded;

	Hysteresis() : raw(NAN), rounded(NAN) {}

	float update(float value)
	{
		raw = value;
		if (std::isnan(rounded))
			return rounded = value;
		int power = pown(10, Precision);
		Scalar hysteresis = (Scalar)Factor / power;
		if (std::abs(value-rounded) > hysteresis)
			rounded = std::roundf(value * power) / power;
		return rounded;
	}
};

#endif // STATS_H
