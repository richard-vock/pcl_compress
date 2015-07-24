#include <compress.hpp>

#include <sstream>
#include <algorithm>
#include <limits>

#include <leptonica/alltypes.h>
#include <leptonica/allheaders.h>
#include <jbig2enc/jbig2enc.h>

#include <openjpeg-2.1/openjpeg.h>

#include <pcl/common/centroid.h>

namespace pcl_compress {

std::vector<PIX*> convert_to_pages(const std::vector<image_t>& images) {
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

chunks_t jbig2_compress_images(const std::vector<image_t>& images) {
    std::vector<PIX*> pages = convert_to_pages(images);

    jbig2ctx* ctx = jbig2_init(0.85f, 0.5f, 0, 0, true, -1);

    chunks_t chunks;
    for (auto& page : pages) {
        chunk_ptr_t chunk(new chunk_t());
        chunk->data = jbig2_encode_generic(page, true, 0, 0, false, &(chunk->length));
        pixDestroy(&page);
        chunks.push_back(chunk);
    }

    jbig2_destroy(ctx);

    return chunks;
}

opj_image_t* convert_for_jpeg2000(const image_t& img) {
    opj_image_cmptparm_t cmptparm[4];
    int numcomps = 1;
    memset(&cmptparm[0], 0, (size_t)numcomps * sizeof(opj_image_cmptparm_t));
    OPJ_COLOR_SPACE color_space = OPJ_CLRSPC_GRAY; //OPJ_CLRSPC_SRGB;
    cv::Size size = img.size();
    int w = size.width;
    int h = size.height;
    for(int i = 0; i < numcomps; i++) {
        cmptparm[i].prec = (OPJ_UINT32)8;
        cmptparm[i].bpp = (OPJ_UINT32)8;
        cmptparm[i].sgnd = 0;
        cmptparm[i].dx = (OPJ_UINT32)1;
        cmptparm[i].dy = (OPJ_UINT32)1;
        cmptparm[i].w = (OPJ_UINT32)w;
        cmptparm[i].h = (OPJ_UINT32)h;
    }

    opj_image_t* image = opj_image_create((OPJ_UINT32)numcomps, &cmptparm[0], color_space);
    image->x0 = 0;
    image->y0 = 0;
    image->x1 = size.width;
    image->y1 = size.height;
    for (int row = 0; row < h; ++row) {
        const uint8_t* row_img = img.ptr(row);
        for (int col = 0; col < w; ++col) {
            for (int i = 0; i < numcomps; ++i) {
                image->comps[i].data[row * w + col] = (OPJ_INT32)row_img[col];
            }
        }
    }

    return image;
}

chunks_t jpeg2000_compress_images(const std::vector<image_t>& images, int quality) {
    opj_cparameters_t parameters;
    opj_set_default_encoder_parameters(&parameters);
    parameters.tcp_distoratio[0] = quality;
    parameters.tcp_numlayers = 1;
    parameters.cp_fixed_quality = 1;
    parameters.numresolution = 2;

    opj_codec_t* codec = 0;
    opj_stream_t* stream = 0;

    chunks_t chunks;
    opj_image_t *image = NULL;
    for (const auto& src_img : images) {
        image = convert_for_jpeg2000(src_img);
        parameters.tcp_mct = (image->numcomps >= 3) ? 1 : 0;

        codec = opj_create_compress(OPJ_CODEC_J2K);

        //opj_set_info_handler(codec, info_callback,00);
        //opj_set_warning_handler(codec, warning_callback,00);
        //opj_set_error_handler(codec, error_callback,00);

        if (! opj_setup_encoder(codec, &parameters, image)) {
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
        opj_stream_set_user_data(stream, (void*)&data_stream, [](void*){});
        opj_stream_set_user_data_length(stream, (OPJ_UINT64)sizeof(data_stream));
        opj_stream_set_write_function(stream, [](void* buf, OPJ_SIZE_T len, void* data) {
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
            std::cerr << "start compress failed" << "\n";
        }
        success = success && opj_encode(codec, stream);
        if (!success) {
            std::cerr << "encode failed" << "\n";
        }
        success = success && opj_end_compress(codec, stream);
        if (!success) {
            std::cerr << "end compress failed" << "\n";
        }
        if (!success) {
            opj_stream_destroy(stream);
            opj_destroy_codec(codec);
            opj_image_destroy(image);
            throw std::runtime_error("Failed to encode image");
        }

        // work on data...
        chunk_ptr_t chunk(new chunk_t());
        data_stream.seekg(0, data_stream.end);
        chunk->length = data_stream.tellg();
        data_stream.seekg(0, data_stream.beg);
        assert(chunk->length > 0);
        chunk->data = (uint8_t*)malloc(chunk->length);
        //chunk->data = new uint8_t[chunk->length];
        data_stream.read((char*)chunk->data, chunk->length);

        opj_stream_destroy(stream);
        opj_destroy_codec(codec);
        opj_image_destroy(image);

        chunks.push_back(chunk);
    }


    return chunks;
}

template <typename PointT>
base_t compute_base_(typename pcl::PointCloud<PointT>::ConstPtr cloud, const std::vector<int>& subset, vec3f_t& centroid) {
    pcl::CentroidPoint<PointT> centroid_computation;
    for (const auto& idx : subset) {
        centroid_computation.add(cloud->points[idx]);
    }
    point_xyz_t c;
    centroid_computation.get(c);
    centroid = c.getVector3fMap();

    Eigen::MatrixXf pos_mat(subset.size(), 3);
    for (uint32_t i = 0; i < subset.size(); ++i) {
        vec3f_t p = cloud->points[subset[i]].getVector3fMap();
        pos_mat.row(i) = (p - centroid).transpose();
    }
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(pos_mat, Eigen::ComputeThinU | Eigen::ComputeThinV);
    base_t base = svd.matrixV().block<3,3>(0,0);
    return base;
}

template <typename PointT>
void project_and_normalize_(typename pcl::PointCloud<PointT>::ConstPtr cloud, const std::vector<int>& subset, patch_t& patch, std::vector<vec3f_t>& projected) {
    base_t project = patch.base.transpose();
    projected.resize(subset.size());
    for (uint32_t i = 0; i < subset.size(); ++i) {
        projected[i] = project * (cloud->points[subset[i]].getVector3fMap() - patch.origin);
        patch.local_bbox.extend(projected[i]);
    }

    vec3f_t min_v = patch.local_bbox.min();
    vec3f_t range = patch.local_bbox.max() - min_v;
    for (auto& v : projected) {
        v = (v - min_v).cwiseQuotient(range);
    }
}

patch_t compute_patch(cloud_xyz_t::ConstPtr cloud, const std::vector<int>& subset, float px_factor, float px_epsilon, const vec2i_t& min_img_size) {
    patch_t patch;

    // compute local base
    patch.base = compute_base_<point_xyz_t>(cloud, subset, patch.origin);

    // project into local base and compute local bounding box
    std::vector<vec3f_t> local_coords;
    project_and_normalize_<point_xyz_t>(cloud, subset, patch, local_coords);

    // compute local resolution
    vec2f_t res = (local_coords[1].head(2) - local_coords[0].head(2)).cwiseAbs();
    vec2f_t eps = vec2f_t(px_epsilon, px_epsilon);
    for (uint32_t i = 0; i < local_coords.size(); ++i) {
        vec3f_t p_i = local_coords[i];
        for (uint32_t j = i+1; j < local_coords.size(); ++j) {
            res = eps.cwiseMax(res.cwiseMin((local_coords[j] - p_i).head(2).cwiseAbs()));
        }
    }

    // compute image extents
    // size = ceil(1 / (factor * res))
    // factor == f  ==>  maximum of f^2 points per pixel on average
    vec2i_t img_size = vec2f_t::Ones().cwiseQuotient(res * px_factor).unaryExpr([] (float x) { return std::ceil(x); }).template cast<int>();
    img_size = img_size.cwiseMax(min_img_size);
    //std::cout << img_size.transpose() << "\n";
    patch.height_map = image_t(img_size[1], img_size[0], CV_8UC1, cv::Scalar(0));
    patch.occ_map = image_t(img_size[1], img_size[0], CV_8UC1, cv::Scalar(0));
    vec2f_t img_size_float = img_size.template cast<float>();
    patch.num_points = 0;
    for (const auto& p : local_coords) {
        vec2i_t uv = p.head(2).cwiseProduct(img_size_float).unaryExpr([] (float x) { return std::floor(x); }).template cast<int>();
        uv[0] = std::min(uv[0], img_size[0]-1);
        uv[1] = std::min(uv[1], img_size[1]-1);

        patch.height_map.at<uint8_t>(uv[1], uv[0]) = static_cast<uint8_t>(p[2] * 255.f);
        if (patch.occ_map.at<uint8_t>(uv[1], uv[0]) == 0) {
            patch.num_points += 1;
        }
        patch.occ_map.at<uint8_t>(uv[1], uv[0]) = uint8_t(255);
    }

    // blur height_map where there is no occupancy
    image_t blurred(img_size[1], img_size[0], CV_8UC1, 0), mask(img_size[1], img_size[0], CV_8UC1);
    cv::subtract(cv::Scalar::all(255), patch.occ_map, mask);
    cv::GaussianBlur(patch.height_map, blurred, cv::Size(9,9), 0);
    blurred.copyTo(patch.height_map, mask);

    return patch;
}

template <typename IntegerType>
IntegerType discretize(float value, float min_value, float max_value) {
    assert(value >= min_value && value <= max_value);
    IntegerType max_rep = std::numeric_limits<IntegerType>::max();
    if (value >= max_value) return max_rep;
    float normalized = (value - min_value) / (max_value - min_value);
    return static_cast<IntegerType>(normalized * static_cast<float>(max_rep));
}


compressed_cloud_t::ptr_t compress_patches(const std::vector<patch_t>& patches, int quality) {
    compressed_cloud_t::ptr_t cloud(new compressed_cloud_t());

    cloud->num_patches = patches.size();
    cloud->num_points = 0;
    for (const auto& patch : patches) {
        cloud->num_points += patch.num_points;
        cloud->bbox_origins.extend(patch.origin);
        cloud->bbox_bboxes.extend(patch.local_bbox.min());
        cloud->bbox_bboxes.extend(patch.local_bbox.max());
    }
    cloud->origins.resize(3 * patches.size());
    cloud->bboxes.resize(6 * patches.size());
    cloud->bases.resize(9 * patches.size());
    uint32_t idx = 0;
    vec3f_t bbmin_o = cloud->bbox_origins.min();
    vec3f_t bbmax_o = cloud->bbox_origins.max();
    vec3f_t bbmin_b = cloud->bbox_bboxes.min();
    vec3f_t bbmax_b = cloud->bbox_bboxes.max();
    std::vector<image_t> height_maps, occ_maps;
    for (const auto& patch : patches) {
        cloud->origins[idx*3+0] = discretize<uint16_t>(patch.origin[0], bbmin_o[0], bbmax_o[0]);
        cloud->origins[idx*3+1] = discretize<uint16_t>(patch.origin[1], bbmin_o[1], bbmax_o[1]);
        cloud->origins[idx*3+2] = discretize<uint16_t>(patch.origin[2], bbmin_o[2], bbmax_o[2]);
        cloud->bboxes[idx*6+0]  = discretize<uint16_t>(patch.local_bbox.min()[0], bbmin_b[0], bbmax_b[0]);
        cloud->bboxes[idx*6+1]  = discretize<uint16_t>(patch.local_bbox.min()[1], bbmin_b[1], bbmax_b[1]);
        cloud->bboxes[idx*6+2]  = discretize<uint16_t>(patch.local_bbox.min()[2], bbmin_b[2], bbmax_b[2]);
        cloud->bboxes[idx*6+3]  = discretize<uint16_t>(patch.local_bbox.max()[0], bbmin_b[0], bbmax_b[0]);
        cloud->bboxes[idx*6+4]  = discretize<uint16_t>(patch.local_bbox.max()[1], bbmin_b[1], bbmax_b[1]);
        cloud->bboxes[idx*6+5]  = discretize<uint16_t>(patch.local_bbox.max()[2], bbmin_b[2], bbmax_b[2]);
        cloud->bases[idx*9+0] = discretize<uint8_t>(patch.base(0, 0), -1.f, 1.f);
        cloud->bases[idx*9+1] = discretize<uint8_t>(patch.base(0, 1), -1.f, 1.f);
        cloud->bases[idx*9+2] = discretize<uint8_t>(patch.base(0, 2), -1.f, 1.f);
        cloud->bases[idx*9+3] = discretize<uint8_t>(patch.base(1, 0), -1.f, 1.f);
        cloud->bases[idx*9+4] = discretize<uint8_t>(patch.base(1, 1), -1.f, 1.f);
        cloud->bases[idx*9+5] = discretize<uint8_t>(patch.base(1, 2), -1.f, 1.f);
        cloud->bases[idx*9+6] = discretize<uint8_t>(patch.base(2, 0), -1.f, 1.f);
        cloud->bases[idx*9+7] = discretize<uint8_t>(patch.base(2, 1), -1.f, 1.f);
        cloud->bases[idx*9+8] = discretize<uint8_t>(patch.base(2, 2), -1.f, 1.f);
        height_maps.push_back(patch.height_map);
        occ_maps.push_back(patch.occ_map);
        std::string outname = "/tmp/out/height_"+std::to_string(idx) + ".png";
        cv::imwrite(outname.c_str(), patch.height_map);
        ++idx;
    }

    chunks_t jbig2_chunks = jbig2_compress_images(occ_maps);
    chunks_t jpeg2k_chunks = jpeg2000_compress_images(height_maps, quality);
    //cloud->global_occ_data = jbig2_stream.global;
    assert(jbig2_chunks.size() == jpeg2k_chunks.size());

    for (uint32_t i = 0; i < jpeg2k_chunks.size(); ++i) {
        std::vector<uint8_t> chunk_jbig2(jbig2_chunks[i]->data, jbig2_chunks[i]->data + jbig2_chunks[i]->length);
        std::vector<uint8_t> chunk_jpeg2k(jpeg2k_chunks[i]->data, jpeg2k_chunks[i]->data + jpeg2k_chunks[i]->length);
        cloud->patch_image_data.push_back(chunk_jbig2);
        cloud->patch_image_data.push_back(chunk_jpeg2k);
    }

    return cloud;
}

} // pcl_compress
