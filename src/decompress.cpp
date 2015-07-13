#include <decompress.hpp>

namespace jbig2 {


std::vector<image_t> naive_decompress(const data_block_t& data_block) {
    const uint32_t* data;
    uint64_t total_bytes;
    std::tie(data, total_bytes) = data_block;

    uint32_t num_images = data[0];
    uint64_t idx = 1, bits_needed = 0;
    std::vector<image_t> images;
    for (uint32_t i = 0; i < num_images; ++i) {
        uint32_t cols = data[idx++];
        uint32_t rows = data[idx++];
        images.push_back(image_t(cols, rows, CV_8UC1));
        bits_needed += cols * rows;
    }

    // first convert word-data to concatenated bit data
    std::vector<bool> concatenated_data(bits_needed, false);
    for (uint64_t first_bit = 0; first_bit < bits_needed; first_bit += 32) {
        uint64_t last_bit = std::min(first_bit + 32, bits_needed);
        uint32_t word = data[idx++];
        bool first_pass = true;//first_bit == 0;
        for (uint64_t bit = 0; bit < (last_bit - first_bit); ++bit) {
            bool val = word & (1 << (32 - bit - 1));
            concatenated_data[first_bit + bit] = val;
        }
        if (first_pass) std::cout << word << "\n";
    }

    idx = 0;
    for (auto& img : images) {
        auto s = img.size();
        for (uint32_t row = 0; row < static_cast<uint32_t>(s.height); ++row) {
            uint8_t* row_ptr = img.ptr(row);
            for (uint32_t col = 0; col < static_cast<uint32_t>(s.width); ++col) {
                *(row_ptr + col) = concatenated_data[idx++] ? 255 : 0;
            }
        }
    }

    return images;
}


} // jbig2
