#ifndef PCL_COMPRESS_COMPRESS_HPP_
#define PCL_COMPRESS_COMPRESS_HPP_

#include "types.hpp"

namespace pcl_compress {

patch_t compute_patch(cloud_normal_t::ConstPtr cloud,
                      const std::vector<int>& subset, const vec2i_t& img_size,
                      uint32_t blur_iters);

compressed_cloud_t::ptr_t
compress_patches(const std::vector<patch_t>& patches, int quality,
                 uint16_t scan_index, const vec3f_t& scan_origin);

}  // pcl_compress

#endif /* PCL_COMPRESS_COMPRESS_HPP_ */
