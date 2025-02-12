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

#ifndef DIFFERENTIATION_H
#define DIFFERENTIATION_H

#ifdef OPT_AUTODIFF
#include "unsupported/Eigen/AutoDiff"
#endif

#include "util/eigendef.hpp"

enum NumericalDiffMode { NumericDiffForward, NumericDiffCentral };

template<typename Scalar, int Mode = NumericDiffForward, bool Parallelise = false, typename Functor>
static inline int NumericDiffJacobian(Functor &&functor, const VectorX<Scalar> &input, Eigen::Ref<MatrixX<Scalar>> jacobian, int eps = 10)
{
	if (jacobian.size() == 0) return 0;

	int ret = 0; // Number of function evaluations
	const Scalar epsilon = std::sqrt(Eigen::NumTraits<Scalar>::epsilon()*eps);

	if constexpr (Parallelise)
	{
		if constexpr (Mode == NumericDiffForward)
		{
			VectorX<Scalar> minVal;
			minVal.resize(jacobian.rows());
			functor(input, minVal); ret++;

		#pragma omp parallel for schedule(dynamic)
			for (int j = 0; j < input.rows(); ++j)
			{
				VectorX<Scalar> maxVal(jacobian.rows());
				VectorX<Scalar> evalInput = input;
				Scalar h = epsilon * abs(input[j]);
				if (h == 0.)
					h = epsilon;
				evalInput[j] += h;
				functor(evalInput, maxVal); ret++;
				evalInput[j] = input[j];
				jacobian.col(j) = (maxVal-minVal)/h;
			}
		}

		/* if constexpr (Mode == NumericDiffForward)
		{
			VectorX<Scalar> minVal;
			minVal.resize(jacobian.rows());
			functor(input, minVal); ret++;

			//VectorX<Scalar> maxVal;
			//VectorX<Scalar> evalInput = input;
		//#pragma omp threadprivate(evalInput, maxVal)

		#pragma omp parallel for schedule(dynamic) //copyin(evalInput)
			for (int j = 0; j < input.rows(); ++j)
			{
				VectorX<Scalar> maxVal(jacobian.rows());
				VectorX<Scalar> evalInput = input;
				//thread_local VectorX<Scalar> evalInput = input;
				//assert(evalInput.size() == jacobian.cols());
				//assert(evalInput(0) == input(0));
				//// This assert DOES fail
				Scalar h = epsilon * abs(input[j]);
				if (h == 0.)
					h = epsilon;
				evalInput[j] += h;
				functor(evalInput, maxVal); ret++;
				evalInput[j] = input[j];
				jacobian.col(j) = (maxVal-minVal)/h;
			}
		} */

		if constexpr (Mode == NumericDiffCentral)
		{
		#pragma omp parallel for schedule(dynamic)
			for (int j = 0; j < input.rows(); ++j)
			{
				thread_local VectorX<Scalar> minVal, maxVal;
				minVal.resize(jacobian.rows());
				maxVal.resize(jacobian.rows());
				VectorX<Scalar> evalInput = input;
				Scalar h = epsilon * abs(input[j]);
				if (h == 0.)
					h = epsilon;
				evalInput[j] += h;
				functor(evalInput, maxVal); ret++;
				evalInput[j] -= 2*h;
				functor(evalInput, minVal); ret++;
				evalInput[j] = input[j];
				jacobian.col(j) = (maxVal-minVal)/(2*h);
			}
		}
	}
	else
	{ // Normal sequential implementation

		VectorX<Scalar> minVal, maxVal;
		minVal.resize(jacobian.rows());
		maxVal.resize(jacobian.rows());

		VectorX<Scalar> evalInput = input;
		if constexpr (Mode == NumericDiffForward)
		{
			functor(evalInput, minVal); ret++;
		}

		for (int j = 0; j < input.rows(); ++j)
		{
			Scalar h = epsilon * abs(input[j]);
			if (h == 0.)
				h = epsilon;
			if constexpr (Mode == NumericDiffForward)
			{
				evalInput[j] += h;
				functor(evalInput, maxVal); ret++;
				evalInput[j] = input[j];
				jacobian.col(j) = (maxVal-minVal)/h;
			}
			else if constexpr (Mode == NumericDiffCentral)
			{
				evalInput[j] += h;
				functor(evalInput, maxVal); ret++;
				evalInput[j] -= 2*h;
				functor(evalInput, minVal); ret++;
				evalInput[j] = input[j];
				jacobian.col(j) = (maxVal-minVal)/(2*h);
			}
		}
	}

	return ret;
}

#ifdef OPT_AUTODIFF

template<typename Scalar>
using AutoDiffScalar = Eigen::AutoDiffScalar<VectorX<Scalar>>;

template<typename Scalar, typename ActiveScalar = AutoDiffScalar<Scalar>, typename Functor>
static inline int AutoDiffJacobian(Functor &&functor, const VectorX<Scalar> &input, Eigen::Ref<MatrixX<Scalar>> jacobian)
{
	VectorX<ActiveScalar> activeParams = input.template cast<ActiveScalar>();
	VectorX<ActiveScalar> activeErrors(jacobian.rows());

	// Initialise output sizes (needed?)
	for (int j = 0; j < jacobian.rows(); j++)
		activeErrors[j].derivatives().resize(input.rows());

	// Initialise input derivatives
	for (int i = 0; i < input.rows(); i++)
		activeParams[i].derivatives() = VectorX<Scalar>::Unit(input.rows(), i);

	int ret = functor(activeParams, activeErrors);

	for (int i = 0; i < jacobian.rows(); i++)
		jacobian.row(i) = activeErrors[i].derivatives();

	return ret;
}
#endif

#endif // DIFFERENTIATION_H