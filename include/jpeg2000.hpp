#ifndef _PCL_COMPRESS_JPEG2000_HPP_
#define _PCL_COMPRESS_JPEG2000_HPP_

#include "types.hpp"

namespace pcl_compress {

chunks_t jpeg2000_compress_images(const std::vector<image_t>& images, int quality);

image_t jpeg2000_decompress_chunk(chunk_const_ptr_t chunk);


} // pcl_compress

#endif /* _PCL_COMPRESS_JPEG2000_HPP_ */
