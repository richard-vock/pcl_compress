#ifndef JBIG2_TYPES_HPP_
#define JBIG2_TYPES_HPP_

#include <vector>
#include <tuple>
#include <opencv2/opencv.hpp>

namespace jbig2 {


typedef cv::Mat image_t;

typedef std::tuple<uint32_t*, uint64_t> data_block_t;


} // jbig2

#endif /* JBIG2_TYPES_HPP_ */
