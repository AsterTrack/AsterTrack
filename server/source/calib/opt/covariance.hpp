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

#include "util/eigendef.hpp"

#ifndef COVARIANCE_PARAMS_H
#define COVARIANCE_PARAMS_H

struct NumericCovarianceParameters
{
	float obsStdDev = 0.08f * PixelSize;
	bool sumObsError = false;
	bool relineariseRotation = false;

	int hessianEstimator = 2;

	// Estimation 0, Samples
	int sampleCount = 1000;
	int sampleGenerator = 1;
	bool sampleRemapGaussian = true;
	int sampleRedistributor = 2;
	// Redistribution
	float sampleRangePos = 0.005f;
	float sampleRangeRot = 0.005f;
	float sampleRangeMin = 0.001f;

	// Numeric Jacobians & Hessians
	float epsHessPos = 0.0001f;
	float epsHessRot = 0.0002f;
	float epsJacPos = 0.00001f;
	float epsJacRot = 0.00003f;

	// Hessian Conditioning
	int hessianConditioning = 1;
	float minHessian = 0.1f;
	float maxHessian = 10.0f;

	template<typename Scalar = float, int DIM = 6>
	Eigen::Vector<Scalar,DIM> getPosRotVec(Scalar pos, Scalar rot) const
	{
		Eigen::Vector<Scalar,DIM> vec = Eigen::Vector<Scalar,DIM>::Constant(pos);
		if constexpr (DIM > 3)
			vec.template tail<DIM-3>().setConstant(rot);
		return vec;
	};
};

#endif

#ifdef INCLUDE_COVARIANCE_PARAMS_ONLY
#undef INCLUDE_COVARIANCE_PARAMS_ONLY
#else

#ifndef COVARIANCE_H
#define COVARIANCE_H

#include "util/log.hpp"

#include "Eigen/Eigenvalues"

template<typename Scalar, int DIM>
static inline void GenerateSamplesRandom(const NumericCovarianceParameters &params, std::vector<std::pair<Eigen::Vector<Scalar, DIM>, Scalar>> &samples)
{
	samples.clear();
	samples.reserve(params.sampleCount);
	Eigen::Vector<Scalar,DIM> evalSample;
	for (int i = 0; i < params.sampleCount; i++)
	{
		for (int j = 0; j < DIM; j++)
			evalSample(j) = (rand()%10000 / 5000.0f - 1.0f);
		samples.emplace_back(evalSample, NAN);
	}
}

template<typename Scalar, int DIM>
static inline void GenerateSamplesUniform(const NumericCovarianceParameters &params, std::vector<std::pair<Eigen::Vector<Scalar, DIM>, Scalar>> &samples)
{
	samples.clear();
	samples.reserve(params.sampleCount);

	static_assert(DIM <= 12);
	static const int PRIMES[12] = { 2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37 };
	auto halton = [](int index, int base)
	{
		double f = 1.0, r = 0.0;
		while (index > 0)
		{
			f /= base;
			r += f * (index % base);
			index /= base;
		}
		return r;
	};

	Eigen::Vector<Scalar,DIM> evalSample;
	for (int i = 0; i < params.sampleCount; i++)
	{
		for (int j = 0; j < DIM; j++)
			evalSample(j) = halton(i, PRIMES[j]) * 2 - 1;
		samples.emplace_back(evalSample, NAN);
	}
}

template<typename Scalar, int DIM>
static inline void RedistributeSamplesGaussian(const NumericCovarianceParameters &params, std::vector<std::pair<Eigen::Vector<Scalar, DIM>, Scalar>> &samples)
{
	auto inverseNormalCDF = [](const Scalar p)
	{
		Scalar r, val;
		const Scalar q = p - 0.5f;

		if (std::abs(q) <= .425) {
			r = .180625 - q * q;
			val =
				q * (((((((r * 2509.0809287301226727 +
					33430.575583588128105) * r + 67265.770927008700853) * r +
					45921.953931549871457) * r + 13731.693765509461125) * r +
					1971.5909503065514427) * r + 133.14166789178437745) * r +
					3.387132872796366608)
				/ (((((((r * 5226.495278852854561 +
					28729.085735721942674) * r + 39307.89580009271061) * r +
					21213.794301586595867) * r + 5394.1960214247511077) * r +
					687.1870074920579083) * r + 42.313330701600911252) * r + 1);
		}
		else {
			if (q > 0) r = 1 - p;
			else r = p;
			r = std::sqrt(-std::log(r));

			if (r <= 5) 
			{
				r += -1.6;
				val = (((((((r * 7.7454501427834140764e-4 +
					.0227238449892691845833) * r + .24178072517745061177) *
					r + 1.27045825245236838258) * r +
					3.64784832476320460504) * r + 5.7694972214606914055) *
					r + 4.6303378461565452959) * r +
					1.42343711074968357734)
					/ (((((((r *
						1.05075007164441684324e-9 + 5.475938084995344946e-4) *
						r + .0151986665636164571966) * r +
						.14810397642748007459) * r + .68976733498510000455) *
						r + 1.6763848301838038494) * r +
						2.05319162663775882187) * r + 1);
			}
			else { /* very close to  0 or 1 */
				r += -5;
				val = (((((((r * 2.01033439929228813265e-7 +
					2.71155556874348757815e-5) * r +
					.0012426609473880784386) * r + .026532189526576123093) *
					r + .29656057182850489123) * r +
					1.7848265399172913358) * r + 5.4637849111641143699) *
					r + 6.6579046435011037772)
					/ (((((((r *
						2.04426310338993978564e-15 + 1.4215117583164458887e-7) *
						r + 1.8463183175100546818e-5) * r +
						7.868691311456132591e-4) * r + .0148753612908506148525)
						* r + .13692988092273580531) * r +
						.59983220655588793769) * r + 1);
			}

			if (q < 0.0) {
				val = -val;
			}
		}

		return val;
	};

	for (auto &sample : samples)
	{
		for (int j = 0; j < DIM; j++)
			sample.first(j) = inverseNormalCDF(std::clamp<Scalar>(sample.first(j), -0.99, 0.99) / 2 + 0.5f);
	}
}

template<typename Scalar, int DIM>
static inline void RedistributeSamplesScale(const NumericCovarianceParameters &params, std::vector<std::pair<Eigen::Vector<Scalar, DIM>, Scalar>> &samples)
{
	Eigen::Vector<Scalar,DIM> sampleRange = params.getPosRotVec<Scalar,DIM>(params.sampleRangePos, params.sampleRangeRot);
	for (auto &sample : samples)
	{
		for (int j = 0; j < DIM; j++)
			sample.first(j) = sample.first(j) * sampleRange(j);
		Scalar normSq = sample.first.squaredNorm();
		if (normSq < params.sampleRangeMin*params.sampleRangeMin)
			sample.first *= params.sampleRangeMin / std::sqrt(normSq);
	}
}

template<typename Scalar, int DIM>
static inline void RedistributeSamplesShell(const NumericCovarianceParameters &params, std::vector<std::pair<Eigen::Vector<Scalar, DIM>, Scalar>> &samples)
{
	Eigen::Vector<Scalar,DIM> sampleRange = params.getPosRotVec<Scalar,DIM>(params.sampleRangePos, params.sampleRangeRot);
	for (auto &sample : samples)
	{
		sample.first = sample.first.normalized();
		for (int j = 0; j < DIM; j++)
			sample.first(j) *= sampleRange(j);
	}
}

template<typename Scalar, int DIM>
static inline void RedistributeSamplesSphere(const NumericCovarianceParameters &params, std::vector<std::pair<Eigen::Vector<Scalar, DIM>, Scalar>> &samples)
{
	Eigen::Vector<Scalar,DIM> sampleRange = params.getPosRotVec<Scalar,DIM>(params.sampleRangePos, params.sampleRangeRot);
	for (auto &sample : samples)
	{
		sample.first = sample.first.normalized();
		for (int j = 0; j < DIM; j++)
			sample.first(j) *= sampleRange(j) * (rand()%10000 / (Scalar)10000);
		Scalar normSq = sample.first.squaredNorm();
		if (normSq < params.sampleRangeMin*params.sampleRangeMin)
			sample.first *= params.sampleRangeMin / std::sqrt(normSq);
	}
}

template<typename Scalar, int DIM, typename Functor>
static inline int SampleErrorFunctor(Functor &&functor, const Eigen::Vector<Scalar, DIM> &input, std::vector<std::pair<Eigen::Vector<Scalar, DIM>, Scalar>> &samples)
{
	int ret = 0; // Number of function evaluations
	float errorBase = functor(input); ret++;
	for (int i = 0; i < samples.size(); i++)
	{
		float errorP = functor(input + samples[i].first); ret++;
		float errorM = functor(input - samples[i].first); ret++;
		samples[i].second = (errorP+errorM)/2.0f - errorBase;
	}
	return ret;
}

template<typename Scalar = float, typename SampleScalar, int DIM>
static Eigen::Matrix<Scalar,DIM,DIM> fitHessianToSamples(const std::vector<std::pair<Eigen::Vector<SampleScalar, DIM>, SampleScalar>> &samples)
{
	if (samples.empty())
		return Eigen::Matrix<Scalar,DIM,DIM>::Zero();

	// Estimate hessian from point samples
	//  for all sample points s: s^T * hessian * s = error
	constexpr int SolveParams = (DIM+1)*DIM/2;
	MatrixX<Scalar> solveMat(samples.size(), SolveParams);
	VectorX<Scalar> errorVec(samples.size());
	for (int i = 0; i < samples.size(); i++)
	{
		auto sample = samples[i].first.template cast<Scalar>();
		int j = 0;
		for (int x = 0; x < DIM; x++)
			for (int y = x; y < DIM; y++)
				solveMat(i,j++) = sample(x)*sample(y);
		errorVec(i) = (Scalar)samples[i].second * 2.0;
	}
	Eigen::Vector<Scalar, SolveParams> params = solveMat.template bdcSvd<Eigen::ComputeThinU | Eigen::ComputeThinV>().solve(errorVec);
	assert(!params.hasNaN());

	// Assemble hessian matrix
	Eigen::Matrix<Scalar,DIM,DIM> hessian;
	int j = 0;
	for (int x = 0; x < DIM; x++)
		for (int y = x; y < DIM; y++)
			hessian(x,y) = hessian(y,x) = params(j++);
	return hessian;
}

template<typename Scalar, int DIM>
static Eigen::Matrix<Scalar,DIM,DIM> covarianceFromHessian(const NumericCovarianceParameters &params, const Eigen::Matrix<Scalar,DIM,DIM> &hessian)
{
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Scalar,6,6>> evd(hessian);
	// Condition eigenvalues (negatives have to be avoided for hessian to be invertible)
	Eigen::Vector<Scalar,6> ev = evd.eigenvalues();
	if (params.hessianConditioning == 1)
		ev = ev.cwiseMax(params.minHessian);
	else if (params.hessianConditioning == 2 && ev.minCoeff() < params.minHessian)
		ev.array() = -ev.minCoeff() + params.minHessian;
	else if (params.hessianConditioning == 3)
	{
		if (ev.minCoeff() < params.minHessian)
			ev.array() = -ev.minCoeff() + params.minHessian;
		ev = ev.cwiseMin(params.maxHessian);
	}
	// Calculate inverse of hessian (via inverse of conditioned eigenvalues)
	Eigen::Matrix<Scalar,6,6> covariance = evd.eigenvectors() * ev.cwiseInverse().asDiagonal() * evd.eigenvectors().transpose();
	// Rescale covariance based on error used to estimate hessian
	if (params.sumObsError == 0 && params.sumObsError)
	{ // Based on error sum
		// TODO: Properly propagate single observation error to covariance via hessian based on error sum
		covariance *= params.obsStdDev*params.obsStdDev;
	}
	else // Based on error mean
		covariance *= params.obsStdDev*params.obsStdDev;
	return covariance;
}

template<typename Scalar, int N>
Eigen::Matrix<Scalar,N,N> sampleCovarianceExtremes(Eigen::Matrix<Scalar,N,N> covariance, float sigma)
{
	// Get "extremes" of covariance, e.g. uncertainty in their primary directions
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Scalar,N,N>> evd(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix<Scalar,N,1> deviations = sigma * evd.eigenvalues().cwiseSqrt();
	return evd.eigenvectors().array().rowwise() * deviations.transpose().array();
}


template<typename Scalar, int N>
Eigen::Matrix<Scalar,N,1> sampleCovarianceUncertainty(Eigen::Matrix<Scalar,N,N> covariance, float sigma, Eigen::Matrix<Scalar,N,N> targetAxis)
{
	// Get "extremes" of covariance, e.g. uncertainty in their primary directions
	Eigen::Matrix<Scalar,N,N> extremes = sampleCovarianceExtremes<Scalar,N>(covariance, sigma);
	// Align extremes to desired axis before sampling the uncertainty
	return (targetAxis * extremes).cwiseAbs().rowwise().norm();
}

#endif // COVARIANCE_H
#endif // INCLUDE_COVARIANCE_PARAMS_ONLY