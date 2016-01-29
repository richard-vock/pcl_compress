#ifndef _PCL_COMPRESS_ZLIB_HPP_
#define _PCL_COMPRESS_ZLIB_HPP_

#include "types.hpp"
#include "cereal.hpp"

namespace pcl_compress {

void zlib_compress_stream(std::istream& input_buffer, std::ostream& output_buffer);

int zlib_decompress_stream(std::istream& input_buffer, std::ostream& output_buffer);

int zlib_decompress_stream_heap(std::istream& input_buffer, std::ostream& output_buffer);

chunk_ptr_t zlib_compress_chunk(chunk_const_ptr_t input);

chunk_ptr_t zlib_decompress_chunk(chunk_const_ptr_t input);

template <typename T>
void zlib_compress_object(T&& v, std::ostream& output_buffer);

template <typename T>
T zlib_decompress_object(std::istream& input_buffer);

} // pcl_compress

#include "zlib.ipp"

#endif /* _PCL_COMPRESS_ZLIB_HPP_ */
