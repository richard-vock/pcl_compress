#include <vector>
#include <pcl/io/pcd_io.h>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include <compress.hpp>
#include <decompress.hpp>
#include <decomposition.hpp>
using namespace pcl_compress;

#include <libunittest/all.hpp>
using namespace unittest::assertions;

COLLECTION(image_compress) {


struct test_context {
    std::vector<image_t> images;
};

struct image_compress : ::unittest::testcase<test_context> {
    static void run() {
        UNITTEST_CLASS(image_compress)

        std::vector<fs::path> image_files;
        fs::directory_iterator dir_it("data/thresholded/"), end_it;
        for (; dir_it != end_it; ++dir_it) {
            fs::path p = *dir_it;
            if (p.extension() == ".jpg") {
                image_files.push_back(p);
            }
        }

        auto context = std::make_shared<test_context>();
        context->images.resize(image_files.size());
        for (uint32_t i = 0; i < image_files.size(); ++i) {
            fs::path p = image_files[i];
            image_t img = cv::imread(p.string(), CV_LOAD_IMAGE_GRAYSCALE);
            auto s = img.size();
            for (uint32_t row = 0; row < static_cast<uint32_t>(s.height); ++row) {
                uint8_t* row_ptr = img.ptr(row);
                for (uint32_t col = 0; col < static_cast<uint32_t>(s.width); ++col) {
                    row_ptr[col] = row_ptr[col] > 188 ? 255 : 0;
                }
            }
            context->images[i] = img;
        }

        UNITTEST_RUNCTX(context, test_jbig2_compress)
        UNITTEST_RUNCTX(context, test_jpeg2000_compress)
    }

    static double compute_rmse(const std::vector<image_t>& images_0, const std::vector<image_t>& images_1) {
        if (images_0.size() != images_1.size()) {
            throw std::runtime_error("Trying to compute RMSE for differently sized image sets");
        }
        if (!images_0.size()) {
            throw std::runtime_error("Trying to compute RMSE for empty image sets");
        }

        double rmse = 0.0;
        for (uint32_t i = 0; i < images_0.size(); ++i) {
            const image_t& a = images_0[i];
            const image_t& b = images_1[i];

            auto s_a = a.size();
            auto s_b = a.size();
            if (s_a.width != s_b.width || s_a.height != s_b.height) {
                throw std::runtime_error("Incompatible images sizes in RMSE computation for image index " + std::to_string(i));
            }

            double img_error = 0.0;
            for (int row = 0; row < s_a.height; ++row) {
                const uint8_t* row_a = a.ptr(row);
                const uint8_t* row_b = b.ptr(row);
                for (int col = 0; col < s_a.width; ++col) {
                    uint8_t val_a = row_a[col];
                    uint8_t val_b = row_b[col];
                    double diff = fabs(static_cast<double>(val_a) - static_cast<double>(val_b));
                    img_error += diff;
                }
            }
            rmse += img_error / static_cast<double>(s_a.height * s_a.width);
        }

        return rmse / static_cast<double>(images_0.size());
    }

    void test_jbig2_compress() {
        auto context = get_test_context();
        uint32_t num_images = context->images.size();
        chunks_t chunks = jbig2_compress_images(context->images);
        assert_equal(num_images, chunks.size());

        std::vector<image_t> reconstructed;
        for (uint32_t i = 0; i < num_images; ++i) {
            reconstructed.push_back(jbig2_decompress_chunk(chunks[i]));
        }
        double rmse = compute_rmse(context->images, reconstructed);
        assert_equal(0.0, rmse);
    }

    void test_jpeg2000_compress() {
        auto context = get_test_context();
        uint32_t num_images = context->images.size();
        chunks_t chunks = jpeg2000_compress_images(context->images, 35);
        assert_equal(num_images, chunks.size());

        std::vector<image_t> reconstructed;
        for (uint32_t i = 0; i < num_images; ++i) {
            reconstructed.push_back(jpeg2000_decompress_chunk(chunks[i]));
        }
        double rmse = compute_rmse(context->images, reconstructed);
        assert_lesser(rmse, 0.8);
    }
};

REGISTER(image_compress)

} // COLLECTION
