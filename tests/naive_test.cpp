#include <iostream>
#include <vector>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include <compress.hpp>
#include <decompress.hpp>

#include <metrics.hpp>

using namespace jbig2;


int main (int argc, char const* argv[]) {
    std::vector<image_t> images;

    for (int i = 1; i < argc; ++i) {
        fs::path p(argv[i]);
        if (fs::exists(p)) {
            std::cout << p << "\n";
            image_t img = cv::imread(p.string(), CV_LOAD_IMAGE_GRAYSCALE);
            images.push_back(img);
        }
    }

    data_block_t compressed = naive_compress(images);

    std::cout << "num bytes: " << std::get<1>(compressed) << "\n";
    std::cout << "num kbytes: " << (std::get<1>(compressed) / 1024) << "\n";

    std::vector<image_t> reconstructed = naive_decompress(compressed);
    delete [] std::get<0>(compressed);

    double rmse = compute_rmse(images, reconstructed);
    std::cout << "RMSE: " << rmse << "\n";

}

