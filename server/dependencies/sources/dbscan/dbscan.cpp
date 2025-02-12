// MIT License, from https://github.com/Eleobert/dbscan/blob/master/dbscan.cpp
// Modified by Seneral to use Eigen directly

#include "dbscan.hpp"

#include "nanoflann/nanoflann.hpp"

#include <vector>

/**
 * Adaptor to a vector of Eigen Vectors/Matrices for nanoflanns' KDTreeSingleIndexAdaptor
*/
template<typename Point>
struct nf_adaptor
{
	const std::vector<Point> &points;

	nf_adaptor(const std::vector<Point> &points) : points(points) {}

	inline std::size_t kdtree_get_point_count() const { return points.size(); }

	inline float kdtree_get_pt(const std::size_t idx, const std::size_t dim) const
	{
		return points[idx][dim];
	}

	auto const *elem_ptr(const std::size_t idx) const
	{
		return points[idx].data();
	}

	template <class BBOX>
	bool kdtree_get_bbox(BBOX &bb) const { return false; }
};

/**
 * Cluster closeby points (pairwise distance below eps)
*/
template<int N, typename Scalar>
std::vector<std::vector<uint32_t>> dbscan(const std::vector<Eigen::Matrix<Scalar,N,1>> &data, Scalar eps, int min_pts)
{
	using Adaptor = nf_adaptor<Eigen::Matrix<Scalar,N,1>>;
	using KDTree = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<Scalar, Adaptor>, Adaptor, N, uint32_t>;

	const Adaptor adapt(data);
	KDTree index(N, adapt, nanoflann::KDTreeSingleIndexAdaptorParams(10));
	index.buildIndex();

	eps *= eps;

	const auto n_points = adapt.kdtree_get_point_count();
	auto visited  = std::vector<bool>(n_points);
	auto clusters = std::vector<std::vector<uint32_t>>();
	auto matches  = std::vector<nanoflann::ResultItem<uint32_t, Scalar>>();
	auto sub_matches = std::vector<nanoflann::ResultItem<uint32_t, Scalar>>();

	for (uint32_t i = 0; i < n_points; i++)
	{
		if (visited[i]) continue;

		index.radiusSearch(adapt.elem_ptr(i), eps, matches, nanoflann::SearchParameters(0.0, false));
		if (matches.size() < static_cast<uint32_t>(min_pts)) continue;
		visited[i] = true;

		std::vector<uint32_t> cluster = {i};

		while (matches.empty() == false)
		{
			auto nb_idx = matches.back().first;
			matches.pop_back();
			if (visited[nb_idx]) continue;
			visited[nb_idx] = true;

			index.radiusSearch(adapt.elem_ptr(nb_idx), eps, sub_matches, nanoflann::SearchParameters(0.0, false));

			if (sub_matches.size() >= static_cast<uint32_t>(min_pts))
			{
				std::copy(sub_matches.begin(), sub_matches.end(), std::back_inserter(matches));
			}
			cluster.push_back(nb_idx);
		}
		clusters.emplace_back(std::move(cluster));
	}
	for (auto &cluster : clusters)
	{
		std::sort(cluster.begin(), cluster.end());
	}
	return clusters;
}

template std::vector<std::vector<uint32_t>> dbscan<2,float>(const std::vector<Eigen::Vector2f> &data, float eps, int min_pts);
template std::vector<std::vector<uint32_t>> dbscan<3,float>(const std::vector<Eigen::Vector3f> &data, float eps, int min_pts);