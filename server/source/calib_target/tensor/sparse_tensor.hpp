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

#ifndef SPARSE_TENSOR_H
#define SPARSE_TENSOR_H

#include "Eigen/Core"

#include <cassert>

/**
 * Sparse Tensor implementation with optimised access to values by each axis
 */
template<typename Scalar, std::size_t Order>
struct SparseTensor
{
	// Updated externally
	std::array<Eigen::Index, Order> dimensions;
	Eigen::Matrix<Scalar,Eigen::Dynamic,1> values;
	Eigen::MatrixXi indices; // (value.size(), Order)

	// Updated internally
	struct OrderMap
	{
		Eigen::VectorXi accumTable; // (dimensions[o]+1)
		Eigen::VectorXi lookupTable; // (values.size())
	};
	std::array<OrderMap, Order> orderMap;

	SparseTensor() {}

	inline Eigen::Index dimension(std::size_t n) const { return dimensions[n]; }
	inline std::size_t entries() const { return values.size(); }
	inline std::size_t size() const { 
		std::size_t sz = 1;
		for (int o = 0; o < Order; o++)
			sz *= dimensions[o];
		return sz;
	 }
	inline Scalar norm() const { return values.norm(); }

	void update()
	{
		// Update order-based indexing structures
		// Very similar to normal Compressed Column/Row storage, except redundant for each axis
		for (int o = 0; o < Order; o++)
		{
			OrderMap &order = orderMap[o];

			// Map each axis to an accumulative index into a lookup table
			order.accumTable = Eigen::VectorXi::Zero(dimension(o)+1);

			// Count values across the orders axis
			for (int i = 0; i < values.size(); i++)
				order.accumTable(indices(i, o))++;

			// Make table cumulative so it can be used as an index
			int accumIndex = 0;
			for (int j = 0; j < dimension(o); j++)
			{
				int count = order.accumTable(j);
				order.accumTable(j) = accumIndex;
				accumIndex += count;
			}
			order.accumTable(dimension(o)) = accumIndex;
			assert(accumIndex == values.size());

			// Create lookup table for cumulative index to map back to source value
			order.lookupTable = Eigen::VectorXi(values.size());
			// Temporary copy to be able to enumerate each index of the orders axis
			Eigen::VectorXi tempIndex = order.accumTable;
			for (int i = 0; i < values.size(); i++)
			{
				int idx = indices(i, o);
				order.lookupTable(tempIndex(idx)) = i;
				tempIndex(idx)++;
			}
		}
	}
};

#endif // SPARSE_TENSOR_H