#include <decompress.hpp>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

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

struct stream_context {
    jbig2dec_params_t params;
    Jbig2GlobalCtx* ctx;

    ~stream_context() {
        jbig2_global_ctx_free(ctx);
    }
};

//std::shared_ptr<stream_context> init_stream_decompress(std::shared_ptr<const chunk_t> global_chunk) {
    //std::shared_ptr<stream_context> context(new stream_context);

    //context->params.mode = render;
    //context->params.verbose = 3;
    //context->params.hash = 0;
    //context->params.output_file = NULL;
    //context->params.output_format = jbig2dec_format_none;
    //Jbig2Ctx* ctx = jbig2_ctx_new(NULL, JBIG2_OPTIONS_EMBEDDED, NULL, NULL, &context->params);
    //jbig2_data_in(ctx, global_chunk->data, global_chunk->length);

    //context->ctx = jbig2_make_global_ctx(ctx);

    //return context;
//}

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

image_t convert_from_jpeg2000(const opj_image_t* img) {
    int w = img->x1;
    int h = img->y1;
    image_t image(h, w, CV_8UC1);
    for (int row = 0; row < h; ++row) {
        uint8_t* row_img = image.ptr(row);
        for (int col = 0; col < w; ++col) {
            row_img[col] = (uint8_t)img->comps[0].data[row * w + col];
        }
    }
    return image;
}

image_t jpeg2000_decompress_chunk(chunk_const_ptr_t chunk) {
	opj_dparameters_t parameters;
	opj_image_t*  image = NULL;
	opj_stream_t* stream = NULL;
	opj_codec_t*  codec = NULL;

	opj_set_default_decoder_parameters(&parameters);

    stream = opj_stream_default_create(1);
    if (!stream){
        throw std::runtime_error("Unable to decompress jpeg2000 image: Cannot create stream.");
    }

    std::stringstream data_stream;
    opj_stream_set_user_data(stream, (void*)&data_stream, [](void*){});
    opj_stream_set_user_data_length(stream, (OPJ_UINT64)chunk->size());
    opj_stream_set_read_function(stream, [](void* buf, OPJ_SIZE_T len, void* data) {
        std::stringstream* str = (std::stringstream*)data;
        str->read((char*)buf, len);
        return len;
    });
    opj_stream_set_seek_function(stream, [](OPJ_OFF_T offset, void* data) {
        std::stringstream* str = (std::stringstream*)data;
        str->seekg(offset, std::ios_base::beg);
        return (int)offset;
    });
    opj_stream_set_skip_function(stream, [](OPJ_OFF_T offset, void* data) -> long int {
        std::stringstream* str = (std::stringstream*)data;
        str->seekg(offset, std::ios_base::cur);
        return (long int)OPJ_TRUE;
    });
    data_stream.write((const char*)chunk->data(), chunk->size());
    data_stream.seekg(0);

    codec = opj_create_decompress(OPJ_CODEC_J2K);

    if (!opj_setup_decoder(codec, &parameters)) {
        opj_stream_destroy(stream);
        opj_destroy_codec(codec);
        throw std::runtime_error("Unable to decompress jpeg2000 image: Cannot setup decoder.");
    }

    if (!opj_read_header(stream, codec, &image)) {
        opj_stream_destroy(stream);
        opj_destroy_codec(codec);
        opj_image_destroy(image);
        throw std::runtime_error("Unable to decompress jpeg2000 image: Cannot read header.");
    }

    if (!(opj_decode(codec, stream, image) && opj_end_decompress(codec,	stream))) {
        opj_destroy_codec(codec);
        opj_stream_destroy(stream);
        opj_image_destroy(image);
        throw std::runtime_error("Unable to decompress jpeg2000 image: Cannot decode image.");
    }

    opj_stream_destroy(stream);
    if (codec) opj_destroy_codec(codec);

    image->color_space = OPJ_CLRSPC_GRAY;
    image_t result = convert_from_jpeg2000(image);
    opj_image_destroy(image);

    return result;
}

cloud_xyz_t::Ptr from_patches(const std::vector<patch_t>& patches) {
    cloud_xyz_t::Ptr cloud(new cloud_xyz_t());

    for (const auto& patch : patches) {
        vec3f_t min_v = patch.local_bbox.min();
        vec3f_t range = patch.local_bbox.max() - min_v;

        cv::Size img_size = patch.occ_map.size();
        vec2f_t img_size_float(static_cast<float>(img_size.width), static_cast<float>(img_size.height));
        for (int row = 0; row < img_size.height; ++row) {
            for (int col = 0; col < img_size.width; ++col) {
                if (patch.occ_map.at<uint8_t>(row, col) == 0) continue;

                vec3f_t local(
                   static_cast<float>(col) / img_size_float[0],
                   static_cast<float>(row) / img_size_float[1],
                   static_cast<float>(patch.height_map.at<uint8_t>(row, col)) / 255.f
                );
                point_xyz_t point;
                point.getVector3fMap() = patch.origin + patch.base * (local.cwiseProduct(range) + min_v);
                cloud->push_back(point);
            }
        }
    }

    return cloud;
}

template <typename IntegerType>
float rescale(IntegerType value, float min_value, float max_value) {
    float max_rep = static_cast<float>(std::numeric_limits<IntegerType>::max());
    return (static_cast<float>(value) / max_rep) * (max_value - min_value) + min_value;
}

std::vector<patch_t> decompress_patches(compressed_cloud_t::const_ptr_t cloud) {
    std::vector<patch_t> patches(cloud->num_patches);

    vec3f_t bbmin_o = cloud->bbox_origins.min();
    vec3f_t bbmax_o = cloud->bbox_origins.max();
    vec3f_t bbmin_b = cloud->bbox_bboxes.min();
    vec3f_t bbmax_b = cloud->bbox_bboxes.max();
    //std::shared_ptr<stream_context> ctx = init_stream_decompress(cloud->global_occ_data);
    uint32_t idx = 0;
    for (auto& patch : patches) {
        patch.origin = vec3f_t(
            rescale<uint16_t>(cloud->origins[idx*3+0], bbmin_o[0], bbmax_o[0]),
            rescale<uint16_t>(cloud->origins[idx*3+1], bbmin_o[1], bbmax_o[1]),
            rescale<uint16_t>(cloud->origins[idx*3+2], bbmin_o[2], bbmax_o[2])
        );
        patch.local_bbox.min() = vec3f_t(
            rescale<uint16_t>(cloud->bboxes[idx*6+0], bbmin_b[0], bbmax_b[0]),
            rescale<uint16_t>(cloud->bboxes[idx*6+1], bbmin_b[1], bbmax_b[1]),
            rescale<uint16_t>(cloud->bboxes[idx*6+2], bbmin_b[2], bbmax_b[2])
        );
        patch.local_bbox.max() = vec3f_t(
            rescale<uint16_t>(cloud->bboxes[idx*6+3], bbmin_b[0], bbmax_b[0]),
            rescale<uint16_t>(cloud->bboxes[idx*6+4], bbmin_b[1], bbmax_b[1]),
            rescale<uint16_t>(cloud->bboxes[idx*6+5], bbmin_b[2], bbmax_b[2])
        );
        patch.base <<
            rescale<uint8_t>(cloud->bases[idx*9+0], -1.f, 1.f),
            rescale<uint8_t>(cloud->bases[idx*9+1], -1.f, 1.f),
            rescale<uint8_t>(cloud->bases[idx*9+2], -1.f, 1.f),
            rescale<uint8_t>(cloud->bases[idx*9+3], -1.f, 1.f),
            rescale<uint8_t>(cloud->bases[idx*9+4], -1.f, 1.f),
            rescale<uint8_t>(cloud->bases[idx*9+5], -1.f, 1.f),
            rescale<uint8_t>(cloud->bases[idx*9+6], -1.f, 1.f),
            rescale<uint8_t>(cloud->bases[idx*9+7], -1.f, 1.f),
            rescale<uint8_t>(cloud->bases[idx*9+8], -1.f, 1.f)
        ;

        chunk_ptr_t chunk_jbig2(new chunk_t(cloud->patch_image_data[idx*2 + 0]));
        chunk_ptr_t chunk_jpeg2k(new chunk_t(cloud->patch_image_data[idx*2 + 1]));

        patch.occ_map = jbig2_decompress_chunk(chunk_jbig2);
        patch.height_map = jpeg2000_decompress_chunk(chunk_jpeg2k);
        std::string img_fn = "/tmp/patch_"+std::to_string(idx)+".png";
        cv::imwrite(img_fn, patch.occ_map);

        ++idx;
    }

    return patches;
}

} // pcl_compress
