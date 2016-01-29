#include <utils.hpp>

namespace pcl_compress {

static const float pi = static_cast<float>(M_PI);

vec2f_t to_normalized_spherical(const vec3f_t& v) {
    return vec2f_t(
        (atan2(v[1], v[0]) + pi) / (2.f * pi),
        acos(v[2]) / pi
    );
}

vec3f_t from_normalized_spherical(const vec2f_t& v) {
    vec2f_t tmp(
        2.f * pi * (v[0]) - pi,
        pi * v[1]
    );
    return vec3f_t(
        cos(tmp[0]) * sin(tmp[1]),
        sin(tmp[0]) * sin(tmp[1]),
        cos(tmp[1])
    );
}

} // pcl_compress
