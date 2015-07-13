#include <compress.hpp>


namespace jbig2 {

data_block_t naive_compress(const std::vector<image_t>& images) {
    uint64_t bits_needed = 0;
    for (const auto& img : images) {
        auto s = img.size();
        bits_needed += s.width * s.height;
    }

    uint64_t words_needed = (bits_needed / 32) + (bits_needed % 32 ? 1 : 0);
    words_needed += 2 * images.size() + 1; // size of each image and number of images

    // first concatenate bit data for easier conversion
    std::vector<bool> concatenated_data(bits_needed, false);
    uint64_t idx = 0;
    for (const auto& img : images) {
        auto s = img.size();
        for (uint32_t row = 0; row < static_cast<uint32_t>(s.height); ++row) {
            const uint8_t* row_ptr = img.ptr(row);
            for (uint32_t col = 0; col < static_cast<uint32_t>(s.width); ++col) {
                concatenated_data[idx++] = row_ptr[col] > 0;
            }
        }
    }

    // prepare data array by first filling it with sizes
    uint32_t* data = new uint32_t[words_needed];
    data[0] = images.size();
    idx = 1;
    for (const auto& img : images) {
        auto s = img.size();
        data[idx++] = s.width;
        data[idx++] = s.height;
    }

    // finally fill data array with data bits word-wise
    for (uint64_t first_bit = 0; first_bit < bits_needed; first_bit += 32) {
        uint64_t last_bit = std::min(first_bit + 32, bits_needed);
        uint32_t word = 0;
        //bool first_pass = true;//first_bit == 0;
        for (uint64_t bit = 0; bit < (last_bit - first_bit); ++bit) {
            bool val = concatenated_data[first_bit + bit];
            if (val) {
                word |= (1 << (32 - bit - 1));
            }
        }
        //if (first_pass) std::cout << word << "\n";
        data[idx++] = word;
    }

    return data_block_t(data, words_needed);
}

} // jbig2
