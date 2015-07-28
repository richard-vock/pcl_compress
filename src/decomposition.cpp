#include <decomposition.hpp>

#include <pcl/octree/octree.h>
#include <primitive_detection/PrimitiveDetector.h>

#include <quadtree.hpp>

namespace pcl_compress {

template <typename PointT>
decomposition_t
octree_decomposition(typename pcl::PointCloud<PointT>::ConstPtr cloud,
                     float leaf_size, ex::optional<subset_t> subset) {
    pcl::octree::OctreePointCloud<PointT> octree(leaf_size);
    boost::shared_ptr<std::vector<int>> index_cloud(new std::vector<int>());
    if (subset) {
        *index_cloud = *subset;
    } else {
        *index_cloud = std::vector<int>(cloud->size());
        std::iota(index_cloud->begin(), index_cloud->end(), 0);
    }
    octree.setInputCloud(cloud, index_cloud);
    octree.addPointsFromInputCloud();

    decomposition_t decomp;
    for (auto it = octree.leaf_begin(); it != octree.leaf_end(); ++it) {
        std::vector<int> leaf_subset;
        it.getLeafContainer().getPointIndices(leaf_subset);
        if (leaf_subset.size() < 5) continue;
        decomp.push_back(leaf_subset);
    }

    return decomp;
}

template <typename PointT>
decomposition_t
primitive_decomposition(typename pcl::PointCloud<PointT>::ConstPtr cloud,
                        const prim_detect_params_t& prim_params,
                        uint32_t max_points_per_cell,
                        uint32_t max_depth,
                        float residual_leaf_size) {
    pcshapes::PrimitiveDetector detector;
    detector.setEpsilon(prim_params.epsilon);
    detector.setBitmapEpsilon(prim_params.bitmap_epsilon);
    detector.setNormalThreshold(1.f - prim_params.angle_threshold);
    detector.setMinimumSupport(prim_params.min_points);
    detector.setProbability(prim_params.probability_threshold);
    pcshapes::SupportedTypes types;
    types.set(pcshapes::PLANE);
    auto primitives = detector.detectPrimitives<PointT>(cloud, types);

    std::set<uint32_t> residual_set, included_set;
    for (uint32_t i = 0; i < cloud->size(); ++i) {
        residual_set.insert(i);
    }

    decomposition_t decomp;
    quadtree::params_t quadtree_params = {
        max_depth,
        max_points_per_cell
    };
    for (auto prim : primitives) {
        auto primPlane =
            std::dynamic_pointer_cast<pcshapes::PrimitivePlane>(prim);
        if (primPlane->area() < prim_params.min_area) continue;
        subset_t indices = primPlane->indices();
        vec3f_t normal = primPlane->normal().normalized();
        vec3f_t bitangent =
            (1.f - fabs(normal[2])) < Eigen::NumTraits<float>::dummy_precision()
                ? vec3f_t::UnitX()
                : vec3f_t::UnitZ();
        vec3f_t tangent = normal.cross(bitangent).normalized();
        bitangent = tangent.cross(normal).normalized();
        base_t local;
        local << tangent, bitangent, normal;
        local.transposeInPlace();
        std::vector<vec2f_t> uv(indices.size());
        for (uint32_t i = 0; i < indices.size(); ++i) {
            uv[i] = (local * cloud->points[indices[i]].getVector3fMap()).head(2);
        }
        quadtree::ptr_t qt(new quadtree(uv, quadtree_params));
        for (const auto& leaf : qt->leaves()) {
            const auto& leaf_indices = leaf.indices();
            if (leaf_indices.size() < 5) continue;
            subset_t global_indices(leaf_indices.size());
            std::transform(leaf_indices.begin(), leaf_indices.end(), global_indices.begin(), [&] (int idx) { return indices[idx]; });
            decomp.push_back(global_indices);
            included_set.insert(global_indices.begin(), global_indices.end());
        }
    }
    // copy residual set while removing all indices included in primitives
    subset_t residual(residual_set.size());
    auto new_end = std::set_difference(residual_set.begin(), residual_set.end(),
                                       included_set.begin(), included_set.end(),
                                       residual.begin());
    residual.resize(std::distance(residual.begin(), new_end));

    // use octree decomposition for all remaining points
    if (residual.size() > 5) {
        //decomp.push_back(residual);
        decomposition_t res_decomp =
            octree_decomposition<PointT>(cloud, residual_leaf_size, residual);
        for (const auto& subset : res_decomp) {
            if (subset.size() > 5) decomp.push_back(subset);
        }
        //decomp.insert(decomp.end(), res_decomp.begin(), res_decomp.end());
    } else {
        std::cout << "no residual" << "\n";
    }

    return decomp;
}

// explicit instantiations
template decomposition_t octree_decomposition<pcl::PointNormal>(
    typename pcl::PointCloud<pcl::PointNormal>::ConstPtr, float,
    ex::optional<subset_t>);
template decomposition_t octree_decomposition<pcl::PointXYZ>(
    typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr, float,
    ex::optional<subset_t>);
template decomposition_t primitive_decomposition<pcl::PointNormal>(
    typename pcl::PointCloud<pcl::PointNormal>::ConstPtr,
    const prim_detect_params_t&, uint32_t, uint32_t, float);

}  // pcl_compress
