#ifndef PCL_COMPRESS_COMPRESS_HPP_
#define PCL_COMPRESS_COMPRESS_HPP_

#include "types.hpp"

namespace pcl_compress {


chunks_t jbig2_compress_images(const std::vector<image_t>& images);

chunks_t jpeg2000_compress_images(const std::vector<image_t>& images, int quality);

//patch_t compute_patch(cloud_xyz_t::ConstPtr cloud, const std::vector<int>& subset, float px_factor, float px_epsilon = Eigen::NumTraits<float>::dummy_precision());
patch_t compute_patch(cloud_xyz_t::ConstPtr cloud, const std::vector<int>& subset, float px_factor, float px_epsilon = Eigen::NumTraits<float>::dummy_precision(), const vec2i_t& min_img_size = vec2i_t(20, 20));

patch_t compute_patch(cloud_normal_t::ConstPtr cloud, const std::vector<int>& subset, const vec2i_t& img_size);

compressed_cloud_t::ptr_t compress_patches(const std::vector<patch_t>& patches, int quality);


} // pcl_compress

#endif /* PCL_COMPRESS_COMPRESS_HPP_ */
