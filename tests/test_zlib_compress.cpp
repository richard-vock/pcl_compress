#include <vector>

#include <zlib.hpp>
using namespace pcl_compress;

#include <libunittest/all.hpp>
using namespace unittest::assertions;

COLLECTION(zlib_compress) {


struct zlib_compress : ::unittest::testcase<> {
    static void run() {
        UNITTEST_CLASS(zlib_compress)

        UNITTEST_RUN(test_zlib_compress)
    }

    void test_zlib_compress() {
        auto context = get_test_context();
        static const uint32_t count = 500000;
        chunk_ptr_t chunk(new chunk_t(count));
        auto rng = unittest::make_random_value<uint8_t>(0, 255);
        rng->seed(23);
        for (uint32_t i = 0; i < count; ++i) {
            uint8_t value = rng->get();
            (*chunk)[i] = value;
        }
        chunk_ptr_t cpr = zlib_compress_chunk(chunk);
        chunk_ptr_t rec = zlib_decompress_chunk(cpr);
        std::cout << cpr->size() << "\n";
        assert_equal_containers(*chunk, *rec);
    }
};

REGISTER(zlib_compress)

} // COLLECTION
