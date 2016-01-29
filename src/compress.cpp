#include <compress.hpp>
#include <utils.hpp>

#include <sstream>
#include <algorithm>
#include <limits>

#include <pcl/common/centroid.h>

#include <jbig2.hpp>
#include <jpeg2000.hpp>
#include <zlib.hpp>

namespace pcl_compress {

template <typename PointT>
base_t
compute_base_(typename pcl::PointCloud<PointT>::ConstPtr cloud,
              const std::vector<int>& subset, vec3f_t& centroid) {
    pcl::CentroidPoint<PointT> centroid_computation;
    for (const auto& idx : subset) {
        centroid_computation.add(cloud->points[idx]);
    }
    point_xyz_t c;
    centroid_computation.get(c);
    centroid = c.getVector3fMap();

    Eigen::MatrixXf pos_mat(subset.size(), 3);
    for (uint32_t i = 0; i < subset.size(); ++i) {
        vec3f_t p = cloud->points[subset[i]].getVector3fMap();
        pos_mat.row(i) = (p - centroid).transpose();
    }
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(
        pos_mat, Eigen::ComputeThinU | Eigen::ComputeThinV);
    base_t base = svd.matrixV().block<3, 3>(0, 0);
    base.col(1) =
        (1.f - fabs(base(2, 2))) < Eigen::NumTraits<float>::dummy_precision()
            ? vec3f_t::UnitX()
            : vec3f_t::UnitZ();
    base.col(0) = base.col(1).cross(base.col(2)).normalized();
    base.col(1) = base.col(2).cross(base.col(0)).normalized();
    return base;
}

template <typename PointT>
void
project_and_normalize_(typename pcl::PointCloud<PointT>::ConstPtr cloud,
                       const std::vector<int>& subset, patch_t& patch,
                       std::vector<vec3f_t>& projected) {
    base_t project = patch.base.transpose();
    projected.resize(subset.size());
    for (uint32_t i = 0; i < subset.size(); ++i) {
        projected[i] = project * (cloud->points[subset[i]].getVector3fMap() -
                                  patch.origin);
        patch.local_bbox.extend(projected[i]);
    }

    vec3f_t min_v = patch.local_bbox.min();
    vec3f_t range = patch.local_bbox.max() - min_v;
    for (auto& v : projected) {
        v = (v - min_v).cwiseQuotient(range);
    }
}

patch_t
compute_patch(cloud_normal_t::ConstPtr cloud, const std::vector<int>& subset,
              const vec2i_t& img_size, uint32_t blur_iters) {
    patch_t patch;

    // compute local base
    patch.base = compute_base_<point_normal_t>(cloud, subset, patch.origin);

    // project into local base and compute local bounding box
    std::vector<vec3f_t> local_coords;
    project_and_normalize_<point_normal_t>(cloud, subset, patch, local_coords);

    patch.height_map =
        image_t(img_size[1], img_size[0], CV_8UC3, cv::Scalar(0));
    patch.occ_map = image_t(img_size[1], img_size[0], CV_8UC1, cv::Scalar(0));
    vec2f_t img_size_float = img_size.template cast<float>();
    patch.num_points = 0;
    uint32_t idx = 0;
    for (const auto& p : local_coords) {
        vec2i_t uv = p.head(2)
                         .cwiseProduct(img_size_float)
                         .unaryExpr([](float x) { return std::floor(x); })
                         .template cast<int>();
        uv[0] = std::min(uv[0], img_size[0] - 1);
        uv[1] = std::min(uv[1], img_size[1] - 1);
        uv[0] = std::max(uv[0], 0);
        uv[1] = std::max(uv[1], 0);
        vec3f_t normal = cloud->points[subset[idx++]].getNormalVector3fMap();
        normal.normalize();
        vec2f_t nrm_sph = to_normalized_spherical(normal);

        patch.height_map.at<cv::Vec3b>(uv[1], uv[0]) =
            cv::Vec3b(static_cast<uint8_t>(p[2] * 255.f),
                      static_cast<uint8_t>(nrm_sph[0] * 255.f),
                      static_cast<uint8_t>(nrm_sph[1] * 255.f));

        if (patch.occ_map.at<uint8_t>(uv[1], uv[0]) == 0) {
            patch.num_points += 1;
        }
        patch.occ_map.at<uint8_t>(uv[1], uv[0]) = uint8_t(255);
    }

    // blur height_map where there is no occupancy
    if (blur_iters) {
        image_t blurred(img_size[1], img_size[0], CV_8UC3, cv::Scalar(0, 0, 0));
        image_t mask(img_size[1], img_size[0], CV_8UC1);
        cv::subtract(cv::Scalar::all(255), patch.occ_map, mask);
        for (uint32_t i = 0; i < blur_iters; ++i) {
            cv::GaussianBlur(patch.height_map, blurred, cv::Size(5, 5), 0);
            blurred.copyTo(patch.height_map, mask);
        }
    }

    return patch;
}

template <typename IntegerType>
IntegerType
discretize(float value, float min_value, float max_value) {
    assert(value >= min_value && value <= max_value);
    IntegerType max_rep = std::numeric_limits<IntegerType>::max();
    if (value >= max_value) return max_rep;
    float normalized = (value - min_value) / (max_value - min_value);
    return static_cast<IntegerType>(normalized * static_cast<float>(max_rep));
}

template <typename T>
void write(std::ostream& out, const T& v) {
    out.write((const char*)&v, sizeof(T));
}

template <>
void write<vec3f_t>(std::ostream& out, const vec3f_t& v) {
    out.write((const char*)&v[0], sizeof(float));
    out.write((const char*)&v[1], sizeof(float));
    out.write((const char*)&v[2], sizeof(float));
}

template <typename IntegerType>
void dwrite(std::ostream& out, const float& v, float min_value, float max_value) {
    IntegerType tmp = discretize<IntegerType>(v, min_value, max_value);
    out.write((const char*)&tmp, sizeof(IntegerType));
}

template <typename IntegerType>
void dwrite(std::ostream& out, const vec3f_t& v, const bbox3f_t& bbox) {
    IntegerType tmp;
    tmp = discretize<IntegerType>(v[0], bbox.min()[0], bbox.max()[0]);
    out.write((const char*)&tmp, sizeof(IntegerType));
    tmp = discretize<IntegerType>(v[1], bbox.min()[1], bbox.max()[1]);
    out.write((const char*)&tmp, sizeof(IntegerType));
    tmp = discretize<IntegerType>(v[2], bbox.min()[2], bbox.max()[2]);
    out.write((const char*)&tmp, sizeof(IntegerType));
}

template <typename IntegerType>
void dwrite(std::ostream& out, const base_t& v, float min_value, float max_value) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            dwrite<IntegerType>(out, v(i,j), min_value, max_value);
        }
    }
}


compressed_cloud_t::ptr_t
compress_patches(const std::vector<patch_t>& patches, int quality,
                 uint16_t scan_index, const vec3f_t& scan_origin) {
    compressed_cloud_t::ptr_t cloud(new compressed_cloud_t());

    std::stringstream gdata;
    write(gdata, scan_origin);
    write(gdata, scan_index);
    uint32_t num_patches = patches.size();
    write(gdata, num_patches);

    //uint32_t num_points = 0;
    bbox3f_t bbox_origins, bbox_bboxes;
    for (const auto& patch : patches) {
        //num_points += patch.num_points;
        bbox_origins.extend(patch.origin);
        bbox_bboxes.extend(patch.local_bbox.min());
        bbox_bboxes.extend(patch.local_bbox.max());
    }
    //write(gdata, num_points);
    write(gdata, bbox_origins.min());
    write(gdata, bbox_origins.max());
    write(gdata, bbox_bboxes.min());
    write(gdata, bbox_bboxes.max());

    std::vector<image_t> height_maps, occ_maps;
    for (const auto& patch : patches) {
        write(gdata, patch.num_points);
        dwrite<uint16_t>(gdata, patch.origin, bbox_origins);
        dwrite<uint16_t>(gdata, patch.local_bbox.min(), bbox_bboxes);
        dwrite<uint16_t>(gdata, patch.local_bbox.max(), bbox_bboxes);
        dwrite<uint8_t>(gdata, patch.base, -1.f, 1.f);

        height_maps.push_back(patch.height_map);
        occ_maps.push_back(patch.occ_map);
    }

    //uint32_t length_global = gdata.tellp();
    gdata.seekg(0);
    std::stringstream gcompr;
    zlib_compress_stream(gdata, gcompr);
    auto compr_length = gcompr.tellp();
    cloud->global_data.resize(compr_length);
    gcompr.seekg(0);
    gcompr.read((char*)cloud->global_data.data(), compr_length);


    chunks_t jbig2_chunks = jbig2_compress_images(occ_maps);
    chunks_t jpeg2k_chunks = jpeg2000_compress_images(height_maps, quality);
    assert(jbig2_chunks.size() == jpeg2k_chunks.size());

    for (uint32_t i = 0; i < jpeg2k_chunks.size(); ++i) {
        cloud->patch_image_data.push_back(*jbig2_chunks[i]);
        cloud->patch_image_data.push_back(*jpeg2k_chunks[i]);
    }

    return cloud;
}

}  // pcl_compress
