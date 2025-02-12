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

#ifndef TENSOR_UTIL_H
#define TENSOR_UTIL_H

#include "Eigen/Core"
#include "unsupported/Eigen/CXX11/Tensor"

#include <cassert>

/**
 * Iterator for multi-dimensional tensors, iterating over multiple axis simultaneously
 * Should perform the same as equivalent nested loops, but slower than a flattened loop with indices pre-computed
 */
template<std::size_t Order>
class TensorIterator : public std::array<Eigen::Index, Order>
{
private:
	typedef std::array<Eigen::Index, Order> IndexArray;
	IndexArray begin, end;

	template<std::size_t O>
	using IndexOrder = std::integral_constant<std::size_t, O>;

	template<std::size_t O = 0>
	void inc(IndexOrder<O> = IndexOrder<O>())
	{
		// Attempt to increase index in current order
		if (++std::get<O>(*this) != std::get<O>(end))
			return;
		// Start increasing index in next order
		std::get<O>(*this) = std::get<O>(begin);
		inc(IndexOrder<O+1>());
	}
	// Increase index in final order, chosen at compile time
	void inc(IndexOrder<Order-1>) { ++std::get<Order-1>(*this); }

public:
	TensorIterator(const IndexArray& b, const IndexArray& e) : IndexArray(b), begin(b), end(e) {}

	TensorIterator& operator++() { return inc(), *this; }
	TensorIterator operator++(int) { TensorIterator ret = *this; ++(*this); return ret; }

	template<typename I>
	Eigen::Matrix<I,Order,1> vector() const
	{
		Eigen::Matrix<I, Order, 1> value;
		for (int i = 0; i < Order; i++) // Will be flattened
			value(i) = this->operator[](i);
		return value;
	}

	operator bool() const { return std::get<Order-1>(*this) != std::get<Order-1>(end); }
};

template<int Mode, typename Scalar, int Order>
static inline Eigen::Map<Eigen::Matrix<Scalar, Eigen::Dynamic, 1>, Eigen::Unaligned, Eigen::InnerStride<>> getFiber(const Eigen::Tensor<Scalar, Order> &tensor, const std::array<Eigen::Index,Order> &index)
{
	typedef Eigen::Map<Eigen::Matrix<Scalar, Eigen::Dynamic, 1>, Eigen::Unaligned, Eigen::InnerStride<>> Map;
	int ModeDim = std::get<Mode>(tensor.dimensions());
	// Determine stride from offset between first two elements of the Mode dimension
	std::array<Eigen::Index, Order> first = index, second = index;
	std::get<Mode>(first) = 0;
	std::get<Mode>(second) = 1;
	std::ptrdiff_t stride = &tensor(second) - &tensor(first);
	// Generate Map
	return Map((Scalar*)&tensor(first), ModeDim, Eigen::InnerStride<>(stride));

}

template<int Mode, typename Scalar, int Order, typename MatrixType>
static inline Eigen::Tensor<Scalar, Order> modeProduct(const Eigen::Tensor<Scalar, Order> &tensor, const MatrixType &matrix)
{
	assert(std::get<Mode>(tensor.dimensions()) == matrix.cols());
	// Iterate over all but the Mode dimension
	std::array<Eigen::Index, Order> ProductDims = tensor.dimensions();
	std::get<Mode>(ProductDims) = 1;
	// Modify Mode dimension to be the new size
	std::array<Eigen::Index, Order> ResultDims = tensor.dimensions();
	std::get<Mode>(ResultDims) = matrix.rows();

	Eigen::Tensor<Scalar, Order> result(ResultDims);
	for (TensorIterator c({}, ProductDims); c; c++)
	{ // Multiply Mode-Fiber with the matrix
		auto fiberTensor = getFiber<Mode, Scalar, Order>(tensor, c);
		auto fiberResult = getFiber<Mode, Scalar, Order>(result, c);
		fiberResult = matrix * fiberTensor;
	}
	return result;
}

template<int Mode, typename Scalar, int Order>
static inline Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> flattenedTensor(const Eigen::Tensor<Scalar, Order> &tensor)
{
	std::array<Eigen::Index, Order> flattenedDims = tensor.dimensions();
	std::get<Mode>(flattenedDims) = 1;
	int flattenedNum = 1;
	for (int o = 0; o < Order; o++)
		flattenedNum *= flattenedDims[o];
	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> result(std::get<Mode>(tensor.dimensions()), flattenedNum);
	int index = 0;
	for (TensorIterator<3> c({}, flattenedDims); c; c++)
		result.col(index++) = getFiber<Mode, Scalar, Order>(tensor, c);
	return result;
}

#endif // TENSOR_UTIL_H 