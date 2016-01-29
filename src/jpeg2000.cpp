#include <jpeg2000.hpp>

#include <openjpeg-2.1/openjpeg.h>

namespace pcl_compress {

opj_image_t*
convert_for_jpeg2000(const image_t& img) {
    opj_image_cmptparm_t cmptparm[4];
    int numcomps = 3;
    memset(&cmptparm[0], 0, (size_t)numcomps * sizeof(opj_image_cmptparm_t));
    OPJ_COLOR_SPACE color_space = OPJ_CLRSPC_SRGB;  // OPJ_CLRSPC_GRAY;
    cv::Size size = img.size();
    int w = size.width;
    int h = size.height;
    for (int i = 0; i < numcomps; i++) {
        cmptparm[i].prec = (OPJ_UINT32)8;
        cmptparm[i].bpp = (OPJ_UINT32)8;
        cmptparm[i].sgnd = 0;
        cmptparm[i].dx = (OPJ_UINT32)1;
        cmptparm[i].dy = (OPJ_UINT32)1;
        cmptparm[i].w = (OPJ_UINT32)w;
        cmptparm[i].h = (OPJ_UINT32)h;
    }

    opj_image_t* image =
        opj_image_create((OPJ_UINT32)numcomps, &cmptparm[0], color_space);
    image->x0 = 0;
    image->y0 = 0;
    image->x1 = size.width;
    image->y1 = size.height;
    for (int row = 0; row < h; ++row) {
        const uint8_t* row_img = img.ptr(row);
        for (int col = 0; col < w; ++col) {
            for (int i = 0; i < numcomps; ++i) {
                image->comps[i].data[row * w + col] =
                    (OPJ_INT32)row_img[col * numcomps + i];
            }
        }
    }

    return image;
}

image_t
convert_from_jpeg2000(const opj_image_t* img) {
    int w = img->x1;
    int h = img->y1;
    const int num_comps = 3;
    image_t image(h, w, CV_8UC3);
    for (int row = 0; row < h; ++row) {
        uint8_t* row_img = image.ptr(row);
        for (int col = 0; col < w; ++col) {
            for (int i = 0; i < num_comps; ++i) {
                row_img[col * num_comps + i] =
                    (uint8_t)img->comps[i].data[row * w + col];
            }
        }
    }
    return image;
}

chunks_t
jpeg2000_compress_images(const std::vector<image_t>& images, int quality) {
    opj_cparameters_t parameters;
    opj_set_default_encoder_parameters(&parameters);
    parameters.tcp_distoratio[0] = quality;
    parameters.tcp_numlayers = 1;
    parameters.cp_fixed_quality = 1;
    parameters.numresolution = 2;

    opj_codec_t* codec = 0;
    opj_stream_t* stream = 0;

    chunks_t chunks;
    opj_image_t* image = NULL;
    for (const auto& src_img : images) {
        image = convert_for_jpeg2000(src_img);
        parameters.tcp_mct = (image->numcomps >= 3) ? 1 : 0;

        codec = opj_create_compress(OPJ_CODEC_J2K);

        // opj_set_info_handler(codec, info_callback,00);
        // opj_set_warning_handler(codec, warning_callback,00);
        // opj_set_error_handler(codec, error_callback,00);

        if (!opj_setup_encoder(codec, &parameters, image)) {
            opj_destroy_codec(codec);
            opj_image_destroy(image);
            throw std::runtime_error("Failed to encode image");
        }

        stream = opj_stream_default_create(0);
        if (!stream) {
            throw std::runtime_error("Unable to create stream");
        }

        // set up stream
        std::stringstream data_stream;
        opj_stream_set_user_data(stream, (void*)&data_stream, [](void*) {});
        opj_stream_set_user_data_length(stream,
                                        (OPJ_UINT64)sizeof(data_stream));
        opj_stream_set_write_function(
            stream, [](void* buf, OPJ_SIZE_T len, void* data) {
                std::stringstream* str = (std::stringstream*)data;
                str->write((const char*)buf, len);
                return len;
            });
        opj_stream_set_seek_function(stream, [](OPJ_OFF_T offset, void* data) {
            std::stringstream* str = (std::stringstream*)data;
            str->seekp(offset);
            return 1;
        });

        bool success = opj_start_compress(codec, image, stream);
        if (!success) {
            std::cerr << "start compress failed"
                      << "\n";
        }
        success = success && opj_encode(codec, stream);
        if (!success) {
            std::cerr << "encode failed"
                      << "\n";
        }
        success = success && opj_end_compress(codec, stream);
        if (!success) {
            std::cerr << "end compress failed"
                      << "\n";
        }
        if (!success) {
            opj_stream_destroy(stream);
            opj_destroy_codec(codec);
            opj_image_destroy(image);
            throw std::runtime_error("Failed to encode image");
        }

        // work on data...
        data_stream.seekg(0, data_stream.end);
        int length = data_stream.tellg();
        data_stream.seekg(0, data_stream.beg);
        assert(length > 0);
        uint8_t* data = new uint8_t[length];
        data_stream.read((char*)data, length);

        chunk_ptr_t chunk(new chunk_t(data, data + length));

        opj_stream_destroy(stream);
        opj_destroy_codec(codec);
        opj_image_destroy(image);

        chunks.push_back(chunk);
    }

    return chunks;
}

image_t
jpeg2000_decompress_chunk(chunk_const_ptr_t chunk) {
    opj_dparameters_t parameters;
    opj_image_t* image = NULL;
    opj_stream_t* stream = NULL;
    opj_codec_t* codec = NULL;

    opj_set_default_decoder_parameters(&parameters);

    stream = opj_stream_default_create(1);
    if (!stream) {
        throw std::runtime_error(
            "Unable to decompress jpeg2000 image: Cannot create stream.");
    }

    std::stringstream data_stream;
    opj_stream_set_user_data(stream, (void*)&data_stream, [](void*) {});
    opj_stream_set_user_data_length(stream, (OPJ_UINT64)chunk->size());
    opj_stream_set_read_function(
        stream, [](void* buf, OPJ_SIZE_T len, void* data) {
            std::stringstream* str = (std::stringstream*)data;
            str->read((char*)buf, len);
            return len;
        });
    opj_stream_set_seek_function(stream, [](OPJ_OFF_T offset, void* data) {
        std::stringstream* str = (std::stringstream*)data;
        str->seekg(offset, std::ios_base::beg);
        return (int)offset;
    });
    opj_stream_set_skip_function(
        stream, [](OPJ_OFF_T offset, void* data) -> long int {
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
        throw std::runtime_error(
            "Unable to decompress jpeg2000 image: Cannot setup decoder.");
    }

    if (!opj_read_header(stream, codec, &image)) {
        opj_stream_destroy(stream);
        opj_destroy_codec(codec);
        opj_image_destroy(image);
        throw std::runtime_error(
            "Unable to decompress jpeg2000 image: Cannot read header.");
    }

    if (!(opj_decode(codec, stream, image) &&
          opj_end_decompress(codec, stream))) {
        opj_destroy_codec(codec);
        opj_stream_destroy(stream);
        opj_image_destroy(image);
        throw std::runtime_error(
            "Unable to decompress jpeg2000 image: Cannot decode image.");
    }

    opj_stream_destroy(stream);
    if (codec) opj_destroy_codec(codec);

    image->color_space = OPJ_CLRSPC_SRGB;  // OPJ_CLRSPC_GRAY;
    image_t result = convert_from_jpeg2000(image);
    opj_image_destroy(image);

    return result;
}

}  // pcl_compress
