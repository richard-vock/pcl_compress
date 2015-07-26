#ifndef PCL_COMPRESS_DECOMPOSITION_HPP_
#define PCL_COMPRESS_DECOMPOSITION_HPP_

#include "types.hpp"

namespace pcl_compress {

typedef std::vector<int> subset_t;
typedef std::vector<subset_t> decomposition_t;

template <typename PointT>
decomposition_t octree_decomposition(typename pcl::PointCloud<PointT>::ConstPtr cloud, float leaf_size);


} // pcl_compress

#endif /* PCL_COMPRESS_DECOMPOSITION_HPP_ */
