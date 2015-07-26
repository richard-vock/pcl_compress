#include <decomposition.hpp>

#include <pcl/octree/octree.h>

namespace pcl_compress {

template <typename PointT>
decomposition_t octree_decomposition(typename pcl::PointCloud<PointT>::ConstPtr cloud, float leaf_size) {
    pcl::octree::OctreePointCloud<PointT> octree(leaf_size);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    decomposition_t decomp;
    for (auto it = octree.leaf_begin(); it != octree.leaf_end(); ++it) {
        std::vector<int> subset;
        it.getLeafContainer().getPointIndices(subset);
        if (subset.size() < 5) continue;
        decomp.push_back(subset);
    }

    return decomp;
}

// explicit instantiations
template decomposition_t octree_decomposition<pcl::PointNormal>(typename pcl::PointCloud<pcl::PointNormal>::ConstPtr, float);
template decomposition_t octree_decomposition<pcl::PointXYZ>(typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr, float);

} // pcl_compress

