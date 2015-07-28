#ifndef PCL_COMPRESS_DECOMPOSITION_HPP_
#define PCL_COMPRESS_DECOMPOSITION_HPP_

#include "types.hpp"

namespace pcl_compress {

typedef std::vector<int> subset_t;
typedef std::vector<subset_t> decomposition_t;

template <typename PointT>
decomposition_t octree_decomposition(
    typename pcl::PointCloud<PointT>::ConstPtr cloud,
    float leaf_size,
    ex::optional<subset_t> subset = ex::nullopt);


typedef struct prim_detect_params_ {
    uint32_t min_points;
    float angle_threshold;
    float epsilon;
    float bitmap_epsilon;
    float min_area;
    float probability_threshold;
} prim_detect_params_t;

template <typename PointT>
decomposition_t primitive_decomposition(
    typename pcl::PointCloud<PointT>::ConstPtr cloud,
    const prim_detect_params_t& prim_params,
    uint32_t max_points_per_cell,
    uint32_t max_depth,
    float residual_leaf_size);


}  // pcl_compress

#endif /* PCL_COMPRESS_DECOMPOSITION_HPP_ */
