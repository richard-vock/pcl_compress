#ifndef PCL_COMPRESS_TYPES_HPP_
#define PCL_COMPRESS_TYPES_HPP_

#include <vector>
#include <tuple>
#include <memory>
#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define USE_CEREAL
#include <cereal.hpp>

namespace pcl_compress {


typedef cv::Mat image_t;

typedef struct chunk_ {
    uint8_t* data;
    int length;

    ~chunk_() {
        free(data);
    }
} chunk_t;

typedef std::shared_ptr<chunk_t> chunk_ptr_t;
typedef std::shared_ptr<const chunk_t> chunk_const_ptr_t;
typedef std::vector<chunk_ptr_t> chunks_t;

typedef struct stream_data_ {
    chunk_ptr_t  global;
    chunks_t     chunks;

    stream_data_() {
        global = std::make_shared<chunk_t>();
    }
} stream_data_t;

typedef pcl::PointXYZ                   point_xyz_t;
typedef pcl::PointNormal                point_normal_t;
typedef pcl::PointCloud<point_xyz_t>    cloud_xyz_t;
typedef pcl::PointCloud<point_normal_t> cloud_normal_t;

typedef Eigen::Vector2i                 vec2i_t;
typedef Eigen::Vector2f                 vec2f_t;
typedef Eigen::Vector3f                 vec3f_t;
typedef Eigen::Matrix3f                 base_t;
typedef Eigen::AlignedBox<float, 3>     bbox3f_t;

typedef struct patch_ {
    uint32_t num_points;
    vec3f_t  origin;
    base_t   base;
    bbox3f_t local_bbox;
    image_t  height_map;
    image_t  occ_map;
    //std::vector<vec3f_t> test;
} patch_t;

typedef struct compressed_cloud_ {
    typedef std::shared_ptr<compressed_cloud_> ptr_t;
    typedef std::shared_ptr<const compressed_cloud_> const_ptr_t;

    // global data
    uint32_t num_patches;
    uint32_t num_points;
    bbox3f_t bbox_origins;
    bbox3f_t bbox_bboxes;
    std::vector<uint16_t> origins;
    std::vector<uint16_t> bboxes;
    std::vector<uint8_t>  bases;
    //chunk_ptr_t global_occ_data;

    // per-patch data
    std::vector<std::vector<uint8_t>> patch_image_data;

    template <typename Archive>
    void serialize(Archive& ar) {
        ar(num_patches);
        ar(num_points);
        ar(bbox_origins);
        ar(bbox_bboxes);
        ar(origins);
        ar(bboxes);
        ar(bases);
        ar(patch_image_data);
    }
} compressed_cloud_t;



} // pcl_compress

#endif /* PCL_COMPRESS_TYPES_HPP_ */
