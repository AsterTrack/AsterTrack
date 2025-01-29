#ifndef DBSCAN_H
#define DBSCAN_H

// MIT license, from https://github.com/Eleobert/dbscan/blob/master/dbscan.hpp
// Modified by Seneral to use Eigen directly

#include "util/eigendef.hpp"

#include <vector>

/**
 * Cluster closeby points (pairwise distance below eps)
*/
template<int N, typename Scalar, typename Index = int>
std::vector<std::vector<Index>> dbscan(const std::vector<Eigen::Matrix<Scalar,N,1>> &data, Scalar eps, int min_pts);

/**
 * Cluster closeby points (pairwise distance below eps)
*/
template<int N, typename Scalar, typename Index = int>
std::vector<std::vector<Index>> dbscanSubset(const std::vector<Eigen::Matrix<Scalar,N,1>> &data, const std::vector<Index> &indices, Scalar eps, int min_pts);

#endif // DBSCAN_H