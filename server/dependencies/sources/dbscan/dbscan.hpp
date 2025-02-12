#ifndef DBSCAN_H
#define DBSCAN_H

// MIT license, from https://github.com/Eleobert/dbscan/blob/master/dbscan.hpp
// Modified by Seneral to use Eigen directly

#include "util/eigendef.hpp"

#include <vector>

/**
 * Cluster closeby points (pairwise distance below eps)
*/
template<int N, typename Scalar>
std::vector<std::vector<uint32_t>> dbscan(const std::vector<Eigen::Matrix<Scalar,N,1>> &data, Scalar eps, int min_pts);

#endif // DBSCAN_H