#ifndef PCL_COMPRESS_DECOMPRESS_HPP_
#define PCL_COMPRESS_DECOMPRESS_HPP_

#include "types.hpp"

namespace pcl_compress {

cloud_normal_t::Ptr from_patches(const std::vector<patch_t>& patches);

global_data_t parse_global_data(std::istream& in);

std::vector<patch_t> decompress_patches(compressed_cloud_t::const_ptr_t, uint16_t& scan_index, vec3f_t& scan_origin);

}  // pcl_compress

#endif /* PCL_COMPRESS_DECOMPRESS_HPP_ */
