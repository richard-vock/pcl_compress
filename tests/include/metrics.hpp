#ifndef PCL_COMPRESS_TESTS_METRICS_HPP_
#define PCL_COMPRESS_TESTS_METRICS_HPP_

#include <types.hpp>

namespace pcl_compress {

double compute_rmse(const std::vector<image_t>& images_0, const std::vector<image_t>& images_1) {
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
        //uint32_t error_count = 0;
        for (int row = 0; row < s_a.height; ++row) {
            const uint8_t* row_a = a.ptr(row);
            const uint8_t* row_b = b.ptr(row);
            for (int col = 0; col < s_a.width; ++col) {
                uint8_t val_a = row_a[col];
                uint8_t val_b = row_b[col];
                double diff = (val_a - val_b) != 0 ? 1.0 : 0.0;
                img_error += diff;
                //if (!!val_a != !!val_b) error_count++;
            }
        }
        //std::cout << "errors: " << error_count << "\n";
        rmse += img_error / static_cast<double>(s_a.height * s_a.width);
    }

    return rmse / static_cast<double>(images_0.size());
}

} // pcl_compress

#endif /* PCL_COMPRESS_TESTS_METRICS_HPP_ */
