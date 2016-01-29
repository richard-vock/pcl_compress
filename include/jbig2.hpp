#ifndef _PCL_COMPRESS_JBIG2_HPP_
#define _PCL_COMPRESS_JBIG2_HPP_

#include "types.hpp"

namespace pcl_compress {

chunks_t jbig2_compress_images(const std::vector<image_t>& images);

image_t jbig2_decompress_chunk(chunk_const_ptr_t chunk);

} // pcl_compress

#endif /* _PCL_COMPRESS_JBIG2_HPP_ */
