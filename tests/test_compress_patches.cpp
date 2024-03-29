#include <vector>
#include <pcl/io/pcd_io.h>

#include <compress.hpp>
#include <decompress.hpp>
#include <decomposition.hpp>
using namespace pcl_compress;

#include <libunittest/all.hpp>
using namespace unittest::assertions;

COLLECTION(compress_patches_test) {


struct test_context {
    std::vector<patch_t> patches;
    compressed_cloud_t::ptr_t cc;
    vec3f_t scan_origin;
};

struct test_compress_patches : ::unittest::testcase<test_context> {
    static void run() {
        UNITTEST_CLASS(test_compress_patches)

        vec2i_t img_size(32, 32);
        cloud_normal_t::Ptr cloud_in(new cloud_normal_t());
        pcl::io::loadPCDFile("data/sampled_house.pcd", *cloud_in);

        bbox3f_t bbox;
        for (const auto& p : cloud_in->points) {
            bbox.extend(p.getVector3fMap());
        }
        prim_detect_params_t params = {
            200,    // min_points
            0.05f,  // angle_threshold
            0.01f,  // epsilon
            0.5f,   // bitmap_epsilon
            0.f,    // min_area
            0.001f  // probability_threshold
        };
        decomposition_t decomp = primitive_decomposition<point_normal_t>(
            cloud_in, params, 1024, 6, 0.03f * bbox.diagonal().norm());

        auto context = std::make_shared<test_context>();
        context->scan_origin = cloud_in->sensor_origin_.head(3);
        for (const auto& subset : decomp) {
            patch_t patch = compute_patch(cloud_in, subset, img_size, 0);
            context->patches.push_back(patch);
        }

        UNITTEST_RUNCTX(context, test_compressed_write)
        UNITTEST_RUNCTX(context, test_compress_decompress)
    }

    void test_compressed_write() {
        auto context = get_test_context();
        context->cc = compress_patches(context->patches, 35, 0, context->scan_origin);
        compressed_cloud_t::ptr_t cc1 = std::make_shared<compressed_cloud_t>();
        serialize(BINARY, "/tmp/cc_write_dump.bin", *context->cc);
        deserialize(BINARY, "/tmp/cc_write_dump.bin", *cc1);

        assert_equal_containers(context->cc->global_data, cc1->global_data);
        assert_equal(context->cc->patch_image_data.size(), cc1->patch_image_data.size());
        for (uint32_t i = 0; i < context->cc->patch_image_data.size(); ++i) {
            assert_equal_containers(context->cc->patch_image_data[i], cc1->patch_image_data[i]);
        }
    }

    void test_compress_decompress() {
        auto context = get_test_context();
        vec3f_t scan_origin;
        uint16_t scan_index;
        std::vector<patch_t> dec = decompress_patches(context->cc, scan_index, scan_origin);

        assert_equal(0, scan_index);
        assert_equal(context->scan_origin[0], scan_origin[0]);
        assert_equal(context->scan_origin[1], scan_origin[1]);
        assert_equal(context->scan_origin[2], scan_origin[2]);

        std::vector<patch_t>& org = context->patches;
        assert_equal(org.size(), dec.size());
        for (uint32_t i = 0; i < org.size(); ++i) {
            float d = (org[i].origin - dec[i].origin).norm();
            assert_lesser(d, 1E-3);

            d = (org[i].base - dec[i].base).norm();
            assert_lesser(d, 2E-2);

            d = (org[i].local_bbox.min() - dec[i].local_bbox.min()).norm();
            d = std::max(d, (org[i].local_bbox.max() - dec[i].local_bbox.max()).norm());
            assert_lesser(d, 1E-3);

            // not comparing images since this is already handled 
            // by the image_compress collection
        }
    }
};

REGISTER(test_compress_patches)

} // COLLECTION
