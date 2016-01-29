#include <jbig2.hpp>

#include <leptonica/alltypes.h>
#include <leptonica/allheaders.h>
#include <jbig2enc/jbig2enc.h>

extern "C" {
#   include <jbig2dec/jbig2.h>
#   include <jbig2dec/jbig2_image.h>
#   include <openjpeg-2.1/openjpeg.h>
}

namespace pcl_compress {

typedef enum {
    usage,dump,render
} jbig2dec_mode;

typedef enum {
    jbig2dec_format_jbig2,
    jbig2dec_format_pbm,
    jbig2dec_format_png,
    jbig2dec_format_none
} jbig2dec_format;

typedef struct {
	jbig2dec_mode mode;
	int verbose, hash;
	char *output_file;
	jbig2dec_format output_format;
} jbig2dec_params_t;

std::vector<PIX*>
convert_to_pages(const std::vector<image_t>& images) {
    std::vector<PIX*> pages;
    for (const auto& img : images) {
        cv::Size s = img.size();
        PIX* pix = pixCreate(s.width, s.height, 1);
        pix->xres = 1;
        pix->yres = 1;

        int wpl = pixGetWpl(pix);
        uint32_t* data = pixGetData(pix);
        for (int row = 0; row < s.height; ++row) {
            uint32_t* row_pix = data + row * wpl;
            const uint8_t* row_img = img.ptr(row);
            for (int col = 0; col < s.width; ++col) {
                if (row_img[col] > 188) {
                    CLEAR_DATA_BIT(row_pix, col);
                } else {
                    SET_DATA_BIT(row_pix, col);
                }
            }
        }

        pages.push_back(pix);
    }

    return pages;
}

chunks_t
jbig2_compress_images(const std::vector<image_t>& images) {
    std::vector<PIX*> pages = convert_to_pages(images);

    jbig2ctx* ctx = jbig2_init(0.85f, 0.5f, 0, 0, true, -1);

    chunks_t chunks;
    for (auto& page : pages) {
        int length;
        uint8_t* data;
        data = jbig2_encode_generic(page, true, 0, 0, false, &length);
        chunk_ptr_t chunk(new chunk_t(data, data + length));
        pixDestroy(&page);
        free(data);
        chunks.push_back(chunk);
    }

    jbig2_destroy(ctx);

    return chunks;
}

image_t jbig2_decompress_chunk(chunk_const_ptr_t chunk) {
    jbig2dec_params_t params;
    params.mode = render;
    params.verbose = 3;
    params.hash = 0;
    params.output_file = NULL;
    params.output_format = jbig2dec_format_none;
    Jbig2Ctx* ctx = jbig2_ctx_new(NULL, (Jbig2Options)0, NULL, NULL, &params);
	jbig2_data_in(ctx, chunk->data(), chunk->size());

    Jbig2Image* rec = jbig2_page_out(ctx);
    image_t img(rec->height, rec->width, CV_8UC1);
    for (int row = 0; row < rec->height; ++row) {
        uint8_t* row_ptr = img.ptr(row);
        for (int col = 0; col < rec->width; ++col) {
            int val = jbig2_image_get_pixel(rec, col, row);
            row_ptr[col] = val > 0 ? 0 : 255;
        }
    }
    jbig2_release_page(ctx, rec);

    jbig2_ctx_free(ctx);

    return img;
}

} // pcl_compress
