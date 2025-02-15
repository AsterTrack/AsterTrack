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

#ifndef COVARIANCE_H
#define COVARIANCE_H

#include "util/eigendef.hpp"

#include "util/log.hpp"

#include "Eigen/Eigenvalues"

template<int DIM, typename Scalar>
static Eigen::Matrix<Scalar,DIM,DIM> fitEllipsoidToSamples(std::vector<VectorX<Scalar>> &samples)
{
	Eigen::Matrix<Scalar,DIM,DIM> ellipsoidMat;
	ellipsoidMat.setZero();
	if (samples.empty()) return ellipsoidMat;

	// Estimate covariance ellipsoid from points samples points on shell
	constexpr int solveParams = (DIM+1)*DIM/2;
	MatrixX<Scalar> solveMat(samples.size(), solveParams); // 72*21=1512 entries for DIM=6...
	for (int i = 0; i < samples.size(); i++)
	{
		auto sample = samples[i].template head<DIM>();
		int j = 0;
		for (int x = 0; x < DIM; x++)
			for (int y = x; y < DIM; y++)
				solveMat(i,j++) = sample(x)*sample(y);
	}
	//VectorX<Scalar> params = solveMat.colPivHouseholderQr().solve(VectorX<Scalar>::Ones(samples.size()));
	VectorX<Scalar> params = solveMat.template bdcSvd<Eigen::ComputeThinU | Eigen::ComputeThinV>().solve(VectorX<Scalar>::Ones(samples.size()));

	// Assemble ellipsoid matrix (for all shell points s: s^T * ellipsoidMat * s = 1)
	int j = 0;
	for (int x = 0; x < DIM; x++)
	{
		for (int y = x; y < DIM; y++)
		{
			Scalar p = params(j++)/2;
			ellipsoidMat(x,y) += p;
			ellipsoidMat(y,x) += p;
		}
	}

	return ellipsoidMat;
}

template<int DIM, typename Scalar>
static Eigen::Matrix<Scalar,DIM,DIM> fitCovarianceToSamples(std::vector<VectorX<Scalar>> &samples)
{
	Eigen::Matrix<Scalar,DIM,DIM> ellipsoidMat = fitEllipsoidToSamples<DIM, Scalar>(samples);

	// Will use general PartialPivLU
	return ellipsoidMat.inverse();

	// TODO: Takes advantage of the fact that ellipsoidMat is symmetric, but probably slower still
	/* Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Scalar,DIM,DIM>> evd(ellipsoidMat, Eigen::ComputeEigenvectors);
	Eigen::Matrix<float,DIM,DIM> axis = evd.eigenvectors();
	Eigen::Matrix<float,DIM,DIM> cov;
	Eigen::Matrix<Scalar,DIM,DIM> covariance;
	covariance.setZero();
	covariance.diagonal() = evd.eigenvalues().cwiseInverse();
	covariance = axis * covariance * axis.transpose();
	return covariance; */
}

template<typename Scalar, bool Parallelise = false, typename Functor>
static inline int NumericCovariance(Functor &&functor, const VectorX<Scalar> &input, Eigen::Ref<MatrixX<Scalar>> covariance,
	Scalar errorStdDev, Scalar errorTolerance, Scalar inputMovement, std::vector<VectorX<Scalar>> &deviations)
{
	int ret = 0; // Number of function evaluations

	Scalar baseError = functor(input); ret++;
	Scalar targetError = baseError + errorStdDev;
	LOG(LOptimisation, LTrace, "    Opt Params yield error of %f, checking variance with target error %f", baseError, targetError);

	auto bisectError = [&](VectorX<Scalar> dir)
	{
		Scalar lower = 0, upper = NAN;
		Scalar errLo = baseError, errUp = NAN;
		Scalar value = 1;
		int it = 0;
		do
		{
			Scalar err = functor(input + dir*value); ret++;
			Scalar diff = std::abs(err-targetError);
			if (diff < errorTolerance) break;
			if (err < targetError)
			{
				lower = value;
				errLo = err;
				if (std::isnan(upper))
					value *= std::lerp(2, targetError/err, 0.0f);
				else
					value = std::lerp((value+upper)/2, std::lerp(value, upper, (targetError-err)/(errUp-err)), 0.0f);
			}
			else
			{
				upper = value;
				errUp = err;
				value = std::lerp((lower+value)/2, std::lerp(lower, value, (targetError-errLo)/(err-errLo)), 0.0f);
			}
		} while (++it < 20);
		if (it > 10)
			LOG(LOptimisation, LDarn, "Bisect Error took %d steps", it);
		return value;
	};

	deviations.clear();
	deviations.reserve(72);
	MatrixX<Scalar> deviationPM(input.size(), 2);
	auto evalDir = input;
	evalDir.setZero();
	for (int i = 0; i < input.size(); i++)
	{
		evalDir(i) = +inputMovement;
		auto facA = bisectError(evalDir);
		deviationPM(i,0) = inputMovement * facA;
		deviations.push_back(evalDir * facA);
		evalDir(i) = -inputMovement;
		auto facB = bisectError(evalDir);
		deviationPM(i,1) = inputMovement * facB;
		deviations.push_back(evalDir * facB);
		evalDir(i) = 0;

		LOG(LOptimisation, LTrace, "        Opt Param %d (%f) has stdDev %f(+) and %f(-)",
			i, input[i], deviationPM(i,0), deviationPM(i,1));
	}

	for (int i = 0; i < input.size(); i++)
	{
		for (int j = i+1; j < input.size(); j++)
		{
			evalDir(i) = +deviationPM(i,0);
			evalDir(j) = +deviationPM(j,0);
			deviations.push_back(evalDir * bisectError(evalDir));

			evalDir(i) = +deviationPM(i,0);
			evalDir(j) = -deviationPM(j,1);
			deviations.push_back(evalDir * bisectError(evalDir));

			evalDir(i) = -deviationPM(i,1);
			evalDir(j) = +deviationPM(j,0);
			deviations.push_back(evalDir * bisectError(evalDir));

			evalDir(i) = -deviationPM(i,1);
			evalDir(j) = -deviationPM(j,1);
			deviations.push_back(evalDir * bisectError(evalDir));

			evalDir(i) = 0;
			evalDir(j) = 0;
		}
	}

	LOG(LOptimisation, LTrace, "    Finished sampling covariance shell with %d evaluations of error!", ret);

	// Estimate covariance ellipsoid from points samples on shell
	covariance = fitCovarianceToSamples<6, float>(deviations);

	return ret;
}

#endif // COVARIANCE_H