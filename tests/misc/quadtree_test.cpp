#include <iostream>
#include <quadtree.hpp>

using namespace pcl_compress;

int main (int argc, char const* argv[]) {
    vec2f_t p0(0.0f, 0.0f);
    vec2f_t p1a(0.5f, 0.0f);
    vec2f_t p1b(0.5f, 0.0f);
    vec2f_t p2a(0.5f, 0.5f);
    vec2f_t p2b(0.5f, 0.5f);
    vec2f_t p2c(0.5f, 0.5f);
    vec2f_t p3a(0.0f, 0.5f);
    vec2f_t p3b(0.0f, 0.5f);
    vec2f_t p3c(0.0f, 0.5f);
    vec2f_t p3d(0.0f, 0.5f);

    std::vector<vec2f_t> points = {p0, p1a, p1b, p2a, p2b, p2c, p3a, p3b, p3c, p3d};
    quadtree::params_t params = {
        1, // max_depth
        4  // max_points_per_cell
    };
    quadtree::ptr_t qt(new quadtree(points, params));

    uint32_t idx = 0;
    for (const auto& node : qt->leaves()) {
        std::cout << (idx++) << " " << node.indices().size() << "\n";
    }
}
