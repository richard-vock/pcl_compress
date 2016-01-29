#ifndef _PCL_COMPRESS_UTILS_HPP_
#define _PCL_COMPRESS_UTILS_HPP_

#include "types.hpp"

namespace pcl_compress {

vec2f_t to_normalized_spherical(const vec3f_t& v);

vec3f_t from_normalized_spherical(const vec2f_t& v);

} // pcl_compress

#endif /* _PCL_COMPRESS_UTILS_HPP_ */
