#include <vector>

#include <utils.hpp>
using namespace pcl_compress;

#include <libunittest/all.hpp>
using namespace unittest::assertions;

COLLECTION(utils) {


struct test_utils : ::unittest::testcase<> {
    static void run() {
        UNITTEST_CLASS(test_utils)
        UNITTEST_RUN(test_spherical)
    }

    void test_spherical() {
        auto random = unittest::make_random_value<float>(-1.f, 1.f);
        random->seed(23);
        float eps = Eigen::NumTraits<float>::dummy_precision();
        for (uint32_t i = 0; i < 1000; ++i) {
            vec3f_t dir;
            dir << random->get(), random->get(), random->get();
            dir.normalize();
            vec2f_t sph = to_normalized_spherical(dir);
            vec3f_t d = from_normalized_spherical(sph);
            assert_approx_equal(dir[0], d[0], eps);
            assert_approx_equal(dir[1], d[1], eps);
            assert_approx_equal(dir[2], d[2], eps);
        }
    }
};

REGISTER(test_utils)

} // COLLECTION
