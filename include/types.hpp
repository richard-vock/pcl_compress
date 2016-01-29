#ifndef PCL_COMPRESS_TYPES_HPP_
#define PCL_COMPRESS_TYPES_HPP_

#include <vector>
#include <tuple>
#include <memory>
#include <experimental/optional>
namespace ex = std::experimental;

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define USE_CEREAL
#include "cereal.hpp"

namespace pcl_compress {


typedef cv::Mat image_t;

typedef std::vector<uint8_t> chunk_t;

typedef std::shared_ptr<chunk_t> chunk_ptr_t;
typedef std::shared_ptr<const chunk_t> chunk_const_ptr_t;
typedef std::vector<chunk_t> stream_t;
typedef std::vector<chunk_ptr_t> chunks_t;

typedef pcl::PointXYZ                   point_xyz_t;
typedef pcl::PointNormal                point_normal_t;
typedef pcl::PointCloud<point_xyz_t>    cloud_xyz_t;
typedef pcl::PointCloud<point_normal_t> cloud_normal_t;

typedef Eigen::Vector2i                 vec2i_t;
typedef Eigen::Vector2f                 vec2f_t;
typedef Eigen::Vector3f                 vec3f_t;
typedef Eigen::Matrix3f                 base_t;
typedef Eigen::AlignedBox<float, 2>     bbox2f_t;
typedef Eigen::AlignedBox<float, 3>     bbox3f_t;

typedef struct patch_ {
    uint32_t num_points;
    vec3f_t  origin;
    base_t   base;
    bbox3f_t local_bbox;
    image_t  height_map;
    image_t  occ_map;
} patch_t;

typedef struct global_data_ {
    vec3f_t scan_origin;
    uint16_t scan_index;
    uint32_t num_patches;
    bbox3f_t bb_o;
    bbox3f_t bb_b;
    std::vector<uint32_t> point_counts;
    std::vector<vec3f_t> origins;
    std::vector<bbox3f_t> bboxes;
    std::vector<base_t> bases;
} global_data_t;

typedef struct merged_global_data_ {
    std::vector<vec3f_t> scan_origins;
    std::vector<uint16_t> scan_indices;
    std::vector<uint32_t> patch_counts;
    std::vector<uint32_t> point_counts;
    std::vector<bbox3f_t> bbs_o;
    std::vector<bbox3f_t> bbs_b;
    std::vector<vec3f_t> origins;
    std::vector<bbox3f_t> bboxes;
    std::vector<base_t> bases;

    template <typename Archive>
    void serialize(Archive& ar) {
        ar(scan_origins);
        ar(scan_indices);
        ar(patch_counts);
        ar(point_counts);
        ar(bbs_o);
        ar(bbs_b);
        ar(origins);
        ar(bboxes);
        ar(bases);
    }
} merged_global_data_t;

typedef struct compressed_cloud_ {
    typedef std::shared_ptr<compressed_cloud_> ptr_t;
    typedef std::shared_ptr<const compressed_cloud_> const_ptr_t;

    chunk_t global_data;
    std::vector<std::vector<uint8_t>> patch_image_data;

    template <typename Archive>
    void serialize(Archive& ar) {
        ar(global_data);
        ar(patch_image_data);
    }
} compressed_cloud_t;


} // pcl_compress

#endif /* PCL_COMPRESS_TYPES_HPP_ */
