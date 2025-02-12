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

#ifndef TUCKER_DECOMPOSITION_H
#define TUCKER_DECOMPOSITION_H

#include "sparse_tensor.hpp"
#include "tensor_util.hpp"

#include <cassert>

#include <Eigen/QR>

/**
 * Iterative Tucker Decomposition for Sparse Tensors
 * Based on P-Tucker https://datalab.snu.ac.kr/ptucker/
 * Completely rewritten with Eigen
 * 
 * Scalar: Scalar to operate with
 * Order: Dimensional Order of the Core Tensor
 * 		If you need Order to be set at runtime, refer to the original implementation
 * Degree: Optional fixed Degree of the core tensor
 * 		Degree == 0:
 * 			Core is arbitrary and mutable (can be orthogonalised)
 *		Degree > 0:
			Core is constrained to a special case and immutable
			The fixed degree specifies the number of entries equal to 1 (the rest is 0)
			You must use the constructor specifying the list of degrees (entries)
			This is a special case implemented for the sake of performance
 */
template<typename Scalar, std::size_t Order, std::size_t Degree = 0>
struct IterativeSparseTuckerDecomposition
{
	typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1> VectorX;
	typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic> MatrixX;
	// Factors may benefit from different memory layout
	//typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic, Eigen::RowMajorBit> MatrixXF;
	typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic> MatrixXF;

	IterativeSparseTuckerDecomposition(std::size_t rank, bool orthogonalise = false) : ortho(orthogonalise)
	{
		assert(Degree == 0);
		std::array<Eigen::Index, Order> core_ranks;
		core_ranks.fill(rank);
		initialiseCore(core_ranks);
	}

	IterativeSparseTuckerDecomposition(const SparseTensor<Scalar, Order> &tensor, std::size_t rank, bool orthogonalise = false) : ortho(orthogonalise)
	{
		assert(Degree == 0);
		std::array<Eigen::Index, Order> core_ranks;
		core_ranks.fill(rank);
		initialiseCore(core_ranks);
		prepareTensor(tensor);
		setRandom();
		compute(tensor);
	}

	IterativeSparseTuckerDecomposition(const std::array<Eigen::Index, Order> &ranks, bool orthogonalise = false) : ortho(orthogonalise)
	{
		assert(Degree == 0);
		initialiseCore(ranks);
	}

	IterativeSparseTuckerDecomposition(const SparseTensor<Scalar, Order> &tensor, const std::array<Eigen::Index, Order> &ranks, bool orthogonalise = false) : ortho(orthogonalise)
	{
		assert(Degree == 0);
		initialiseCore(ranks);
		prepareTensor(tensor);
		setRandom();
		compute(tensor);
	}

	IterativeSparseTuckerDecomposition(const std::array<Eigen::Index, Order> &ranks, const std::array<std::array<Eigen::Index, Order>, Degree> &degrees) : ortho(false)
	{
		assert(Degree > 0);
		factorDegrees = degrees;
		initialiseCore(ranks);
	}

	// Access core (const, non-const) and factors (const, non-const)(compile-time index, runtime index)(matrix, tensor)

	inline Eigen::Tensor<Scalar, Order> &coreTensor() { assert(Degree == 0); return core; }
	inline MatrixXF &factor(Eigen::Index n) { return factors[n]; }
	inline Eigen::TensorMap<Eigen::Tensor<Scalar,2>> factorTensor(Eigen::Index n)
		{ return Eigen::TensorMap<Eigen::Tensor<Scalar,2>>((Scalar*)factors[n].data(), factors[n].rows(), factors[n].cols()); }
	template<std::size_t N> inline MatrixXF &factor()
		{ return std::get<N>(factors); }
	template<std::size_t N> inline Eigen::TensorMap<Eigen::Tensor<Scalar,2>> factorTensor()
		{ return Eigen::TensorMap<Eigen::Tensor<Scalar,2>>((Scalar*)std::get<N>(factors).data(), std::get<N>(factors).rows(), std::get<N>(factors).cols()); }

	inline const Eigen::Tensor<Scalar, Order> &coreTensor() const { return core; }
	inline const MatrixXF &factor(Eigen::Index n) const { return factors[n]; }
	inline const Eigen::TensorMap<Eigen::Tensor<Scalar,2>> factorTensor(Eigen::Index n) const
		{ return Eigen::TensorMap<Eigen::Tensor<Scalar,2>>((Scalar*)factors[n].data(), factors[n].rows(), factors[n].cols()); }
	template<std::size_t N> inline const MatrixXF &factor() const
		{ return std::get<N>(factors); }
	template<std::size_t N> inline const Eigen::TensorMap<Eigen::Tensor<Scalar,2>> factorTensor() const
		{ return Eigen::TensorMap<Eigen::Tensor<Scalar,2>>((Scalar*)std::get<N>(factors).data(), std::get<N>(factors).rows(), std::get<N>(factors).cols()); }

	inline Scalar getRMSE() const { return result.first; }
	inline Scalar getFit() const { return result.second; }
	inline bool orthogonalisationEnabled() const { return ortho; }
	inline bool isCoreMutable() const { return Degree == 0; }

	inline void prepareTensor(const SparseTensor<Scalar, Order> &tensor)
	{
		// Initialise factors
		for (int o = 0; o < Order; o++)
			factors[o] = MatrixX(tensor.dimension(o), core.dimension(o));

		// Enter occupied indices of each axis into queue (for better parallelisation)
		for (int o = 0; o < Order; o++)
		{
			auto &order = tensor.orderMap[o];
			queue[o].clear();
			for (int i = 0; i < tensor.dimension(o); i++)
			{ // Queue indices with non-zero values
				int count_nz = order.accumTable(i + 1) - order.accumTable(i);
				if (count_nz > 0)
					queue[o].push_back(i);
			}
		}
	}

	inline void setRandom()
	{
		// Set arbitrary core (overdetermined, won't change)		
		if constexpr (Degree == 0)
			core.setRandom();

		// Initialise factors with random values
		for (int o = 0; o < Order; o++)
			factors[o].setRandom();
	}

	inline Scalar operator() (Eigen::Matrix<int,1,Order> indices) const
	{
		Scalar result = 0;
		if constexpr (Degree == 0)
		{ // Use mutable core
			for (int c = 0; c < core.size(); c++)
			{ // Could also iterate over core with TensorIterator, but flattened loop /w coreIndices performs better
				Scalar cur = core.data()[c];
				for (int o = 0; o < Order; o++)
					cur *= factors[o](indices(o), coreIndices(c, o));
				result += cur;
			}
		}
		else
		{ // Use immutable core with fixed number of 1s (rest 0)
			for (int d = 0; d < factorDegrees.size(); d++)
			{
				Scalar cur = 1;
				for (int o = 0; o < Order; o++)
					cur *= factors[o](indices(o), factorDegrees[d][o]);
				result += cur;
			}
		}
		return result;
	}

	std::pair<Scalar,Scalar> compute(const SparseTensor<Scalar, Order> &tensor, int max_iterations = 20, Scalar epsilon = 0.0001, Scalar lambda = 0.001)
	{
		result = getReconstructionError(tensor);
		for (int i = 0; i < max_iterations; i++)
		{
			for (int o = 0; o < Order; o++)
				updateFactor(o, tensor, lambda);

			std::pair<Scalar,Scalar> it = getReconstructionError(tensor);

			if (i > 0 && std::abs(result.second-it.second) <= epsilon)
				break;

			result = it;
		}

		if (ortho)
		{
			assert(Degree == 0);
			orthogonalise();
			result = getReconstructionError(tensor);
		}

		return result;
	}

	void updateFactor(int o, const SparseTensor<Scalar, Order> &tensor, Scalar lambda = 0.001)
	{ // Update the o-th factor matrix
		auto &order = tensor.orderMap[o];

	#pragma omp parallel for schedule(dynamic)
		for (int q = 0; q < queue[o].size(); q++)
		{ // Update the i-th row of o-th factor matrix
			int i = queue[o][q]; // in tensor.dimension(o)

			Eigen::Index rank = core.dimension(o);
			VectorX delta(rank);
			MatrixX B = lambda * MatrixX::Identity(rank, rank);
			VectorX C = VectorX::Zero(rank);

			for (int n = order.accumTable(i); n < order.accumTable(i+1); n++)
			{ // Iterate over tensor values with the same index in this order axis
				int idx = order.lookupTable(n);

				// Determine delta
				delta.setZero();
				if constexpr (Degree == 0)
				{ // Use mutable core
					for (int c = 0; c < core.size(); c++)
					{ // Could also iterate over core with TensorIterator, but flattened loop performs better
						Scalar result = core.data()[c];
						for (int oo = 0; oo < Order; oo++)
						{
							if (oo != o)
								result *= factors[oo](tensor.indices(idx, oo), coreIndices(c, oo));
						}
						delta[coreIndices(c, o)] += result;
					}
				}
				else
				{ // Use immutable core with fixed number of 1s (rest 0)
					for (int d = 0; d < factorDegrees.size(); d++)
					{
						Scalar result = 1;
						for (int oo = 0; oo < Order; oo++)
						{
							if (oo != o)
								result *= factors[oo](tensor.indices(idx, oo), factorDegrees[d][oo]);
						}
						delta[factorDegrees[d][o]] += result;
					}
				}

				// Update B and C with delta
				for (int r = 0; r < rank; r++)
					B.col(r) += delta[r] * delta;
				C += tensor.values[idx] * delta;
			}

			// Update the i-th row of o-th factor matrix with B solved for C
			factors[o].row(i) = B.householderQr().solve(C);
		}
	}

	std::pair<Scalar,Scalar> getReconstructionError(const SparseTensor<Scalar, Order> &tensor)
	{
		VectorX errorVec = tensor.values;
	#pragma omp parallel for schedule(static)
		for (int i = 0; i < tensor.values.size(); i++)
		{
			errorVec(i) -= operator()(tensor.indices.row(i));
		}
		Scalar errorSq = errorVec.squaredNorm();
		Scalar RMSE = std::sqrt(errorSq / errorVec.size());
		Scalar fit = 1;
		if (tensor.norm() != 0)
			fit = 1 - std::sqrt(errorSq) / tensor.norm();
		return { RMSE, fit };
	}

	void orthogonalise()
	{
		if constexpr (Degree > 0)
			return; // Cannot orthogonalise with core being immutable
		for (int o = 0; o < Order; o++)
		{ // Update each axis individually

			// Update factor
			Eigen::HouseholderQR<MatrixXF> qr(factors[o]);
			MatrixXF thinQ = qr.householderQ() * MatrixXF::Identity(factors[o].rows(), factors[o].cols());
			MatrixXF R = thinQ.transpose()*factors[o];
			factors[o] = thinQ;

			// Update core
			Eigen::Tensor<Scalar, Order> newCore = core;
			newCore.setZero();
			for (TensorIterator c({}, core.dimensions()); c; c++)
			{ // Flattened loop would be unreadable since other values along one axis are accessed
				Scalar cur = core(c);
				for (int r = 0; r < core.dimension(o); r++)
				{ // Update other values along this axis
					auto pos = c;
					pos[o] = r;
					newCore(pos) += core(c) * R(r, c[o]);
				}
			}
			core = newCore;
		}
	}

private:

	Eigen::Tensor<Scalar, Order> core;  // Read-Only if Degree > 0
	Eigen::MatrixXi coreIndices; // For Degree == 0
	std::array<std::array<Eigen::Index, Order>, Degree> factorDegrees;  // For Degree > 0
	std::array<MatrixXF, Order> factors;
	std::array<std::vector<int>, Order> queue;
	bool ortho; // For Degree == 0

	std::pair<Scalar,Scalar> result;
	
	void initialiseCore(const std::array<Eigen::Index, Order> &ranks)
	{
		// Initialise Core
		core = Eigen::Tensor<Scalar, Order>(ranks);
		
		if constexpr (Degree == 0)
		{
			// Initialise coreIndices used to iterate over flattened core
			coreIndices = Eigen::MatrixXi(core.size(), Order);
			for (TensorIterator c({}, core.dimensions()); c; c++)
			{ // Map flattened index to tensor index for quicker iterations over core
				int offset = &core(c) - core.data();
				for (int o = 0; o < Order; o++)
					coreIndices(offset, o) = c[o];
			}
		}
		else
		{ // Setup core based on iterator factorDegrees
			core.setZero();
			for (int d = 0; d < factorDegrees.size(); d++)
			{
				for (int o = 0; o < Order; o++)
					core(factorDegrees[d][o]) = 1;
			}
			
		}
	}
};

#endif // TUCKER_DECOMPOSITION_H