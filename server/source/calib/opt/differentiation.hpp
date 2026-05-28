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

template<typename Scalar, int Mode = NumericDiffForward, bool Parallelise = false, bool RelativeEps = false, typename Functor, typename TI, typename E = Scalar>
static inline int NumericDiffJacobian(Functor &&functor, const Eigen::MatrixBase<TI> &input, Eigen::Ref<MatrixX<Scalar>> jacobian, E eps = std::sqrt(Eigen::NumTraits<Scalar>::epsilon()))
{
	static_assert(std::is_same<Scalar, typename TI::Scalar>::value);
	if (jacobian.size() == 0) return 0;

	int ret = 0; // Number of function evaluations
	auto epsilon = [&](int i)
	{
		Scalar h;
		if constexpr (std::is_scalar_v<E>)
			h = (Scalar)eps;
		else
			h = (Scalar)eps(i);
		if constexpr (RelativeEps)
			return h * abs(input(i));
		return h;
	};

	if constexpr (Parallelise)
	{
		if constexpr (Mode == NumericDiffForward)
		{
			typename TI::PlainObject baseInput(input);
			VectorX<Scalar> minVal;
			minVal.resize(jacobian.rows());
			functor(baseInput, minVal); ret++;

		#pragma omp parallel for schedule(dynamic)
			for (int i = 0; i < input.rows(); i++)
			{
				VectorX<Scalar> maxVal(jacobian.rows());
				typename TI::PlainObject evalInput = input;
				Scalar h = epsilon(i);
				evalInput[i] += h;
				functor(evalInput, maxVal); ret++;
				evalInput[i] = input[i];
				jacobian.col(i) = (maxVal-minVal)/h;
			}
		}

		/* if constexpr (Mode == NumericDiffForward)
		{
			VectorX<Scalar> minVal;
			minVal.resize(jacobian.rows());
			functor(input, minVal); ret++;

			//VectorX<Scalar> maxVal;
			//typename TI::PlainObject evalInput = input;
		//#pragma omp threadprivate(evalInput, maxVal)

		#pragma omp parallel for schedule(dynamic) //copyin(evalInput)
			for (int i = 0; i < input.rows(); i++)
			{
				VectorX<Scalar> maxVal(jacobian.rows());
				typename TI::PlainObject evalInput = input;
				//thread_local typename TI::PlainObject evalInput = input;
				//assert(evalInput.size() == jacobian.cols());
				//assert(evalInput(0) == input(0));
				//// This assert DOES fail
				Scalar h = epsilon(i);
				evalInput[i] += h;
				functor(evalInput, maxVal); ret++;
				evalInput[i] = input[i];
				jacobian.col(i) = (maxVal-minVal)/h;
			}
		} */

		if constexpr (Mode == NumericDiffCentral)
		{
		#pragma omp parallel for schedule(dynamic)
			for (int i = 0; i < input.rows(); i++)
			{
				thread_local VectorX<Scalar> minVal, maxVal;
				minVal.resize(jacobian.rows());
				maxVal.resize(jacobian.rows());
				typename TI::PlainObject evalInput = input;
				Scalar h = epsilon(i);
				evalInput[i] += h;
				functor(evalInput, maxVal); ret++;
				evalInput[i] -= 2*h;
				functor(evalInput, minVal); ret++;
				evalInput[i] = input[i];
				jacobian.col(i) = (maxVal-minVal)/(2*h);
			}
		}
	}
	else
	{ // Normal sequential implementation

		VectorX<Scalar> minVal, maxVal;
		minVal.resize(jacobian.rows());
		maxVal.resize(jacobian.rows());

		typename TI::PlainObject evalInput = input;
		if constexpr (Mode == NumericDiffForward)
		{
			functor(evalInput, minVal); ret++;
		}

		for (int i = 0; i < input.rows(); i++)
		{
			Scalar h = epsilon(i);
			if constexpr (Mode == NumericDiffForward)
			{
				evalInput[i] += h;
				functor(evalInput, maxVal); ret++;
				evalInput[i] = input[i];
				jacobian.col(i) = (maxVal-minVal)/h;
			}
			else if constexpr (Mode == NumericDiffCentral)
			{
				evalInput[i] += h;
				functor(evalInput, maxVal); ret++;
				evalInput[i] -= 2*h;
				functor(evalInput, minVal); ret++;
				evalInput[i] = input[i];
				jacobian.col(i) = (maxVal-minVal)/(2*h);
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
	for (int i = 0; i < jacobian.rows(); i++)
		activeErrors[i].derivatives().resize(input.rows());

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