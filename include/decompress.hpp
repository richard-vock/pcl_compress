#ifndef JBIG2_DECOMPRESS_HPP_
#define JBIG2_DECOMPRESS_HPP_

#include "types.hpp"

namespace jbig2 {

std::vector<image_t> naive_decompress(const data_block_t& data_block);

} // jbig2

#endif /* JBIG2_DECOMPRESS_HPP_ */
