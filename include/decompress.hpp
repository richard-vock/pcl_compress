#ifndef PCL_COMPRESS_DECOMPRESS_HPP_
#define PCL_COMPRESS_DECOMPRESS_HPP_

#include "types.hpp"

namespace pcl_compress {

struct stream_context;

//std::shared_ptr<stream_context> init_stream_decompress(std::shared_ptr<const chunk_t> global_chunk);

image_t jbig2_decompress_chunk(chunk_const_ptr_t chunk);

image_t jpeg2000_decompress_chunk(chunk_const_ptr_t chunk);

cloud_xyz_t::Ptr from_patches(const std::vector<patch_t>& patches);

std::vector<patch_t> decompress_patches(compressed_cloud_t::const_ptr_t);


} // pcl_compress

#endif /* PCL_COMPRESS_DECOMPRESS_HPP_ */
