#include <decompress.hpp>
#include <utils.hpp>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include <jbig2.hpp>
#include <jpeg2000.hpp>
#include <zlib.hpp>

namespace pcl_compress {

cloud_normal_t::Ptr from_patches(const std::vector<patch_t>& patches) {
    cloud_normal_t::Ptr cloud(new cloud_normal_t());

    for (const auto& patch : patches) {
        vec3f_t min_v = patch.local_bbox.min();
        vec3f_t range = patch.local_bbox.max() - min_v;

        cv::Size img_size = patch.occ_map.size();
        vec2f_t img_size_float(static_cast<float>(img_size.width),
                               static_cast<float>(img_size.height));
        for (int row = 0; row < img_size.height; ++row) {
            for (int col = 0; col < img_size.width; ++col) {
                if (patch.occ_map.at<uint8_t>(row, col) == 0) continue;

                cv::Vec3b hmap_val = patch.height_map.at<cv::Vec3b>(row, col);
                vec3f_t local(
                   static_cast<float>(col) / img_size_float[0],
                   static_cast<float>(row) / img_size_float[1],
                   static_cast<float>(hmap_val[0]) / 255.f
                );
                point_normal_t point;
                point.getVector3fMap() =
                    patch.origin +
                    patch.base * (local.cwiseProduct(range) + min_v);

                vec2f_t nrm_sph(static_cast<float>(hmap_val[1]) / 255.f,
                                static_cast<float>(hmap_val[2]) / 255.f);
                point.getNormalVector3fMap() =
                    from_normalized_spherical(nrm_sph);
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

template <typename T>
void read(std::istream& in, T& v) {
    in.read((char*)&v, sizeof(T));
}

template <>
void read<vec3f_t>(std::istream& in, vec3f_t& v) {
    in.read((char*)&v[0], sizeof(float));
    in.read((char*)&v[1], sizeof(float));
    in.read((char*)&v[2], sizeof(float));
}

template <typename T>
T read(std::istream& in) {
    T v;
    in.read((char*)&v, sizeof(T));
    return v;
}

template <>
vec3f_t read<vec3f_t>(std::istream& in) {
    vec3f_t v;
    in.read((char*)&v[0], sizeof(float));
    in.read((char*)&v[1], sizeof(float));
    in.read((char*)&v[2], sizeof(float));
    return v;
}

template <typename IntegerType>
void dread(std::istream& in, vec3f_t& v, const bbox3f_t& bbox) {
    IntegerType tmp;
    read(in, tmp);
    v[0] = rescale<IntegerType>(tmp, bbox.min()[0], bbox.max()[0]);
    read(in, tmp);
    v[1] = rescale<IntegerType>(tmp, bbox.min()[1], bbox.max()[1]);
    read(in, tmp);
    v[2] = rescale<IntegerType>(tmp, bbox.min()[2], bbox.max()[2]);
}

template <typename IntegerType>
void dread(std::istream& in, base_t& v, float min_value, float max_value) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            v(i,j) = rescale<IntegerType>(read<IntegerType>(in), min_value, max_value);
        }
    }
}

global_data_t parse_global_data(std::istream& in) {
    global_data_t data;

    read(in, data.scan_origin);
    read(in, data.scan_index);
    data.num_patches = read<uint32_t>(in);
    //data.num_points = read<uint32_t>(in);
    data.bb_o.min() = read<vec3f_t>(in);
    data.bb_o.max() = read<vec3f_t>(in);
    data.bb_b.min() = read<vec3f_t>(in);
    data.bb_b.max() = read<vec3f_t>(in);

    data.point_counts.resize(data.num_patches);
    data.origins.resize(data.num_patches);
    data.bboxes.resize(data.num_patches);
    data.bases.resize(data.num_patches);
    for (uint32_t i = 0; i < data.num_patches; ++i) {
        read<uint32_t>(in, data.point_counts[i]);
        dread<uint16_t>(in, data.origins[i], data.bb_o);
        dread<uint16_t>(in, data.bboxes[i].min(), data.bb_b);
        dread<uint16_t>(in, data.bboxes[i].max(), data.bb_b);
        dread<uint8_t>(in, data.bases[i], -1.f, 1.f);
    }

    return data;
}

std::vector<patch_t> decompress_patches(compressed_cloud_t::const_ptr_t cloud, uint16_t& scan_index, vec3f_t& scan_origin) {
    std::stringstream gcompr, gdata;
    gcompr.write((const char*)cloud->global_data.data(), cloud->global_data.size());
    gcompr.seekg(0);
    zlib_decompress_stream(gcompr, gdata);
    gdata.seekg(0);

    global_data_t g = parse_global_data(gdata);
    scan_index = g.scan_index;
    scan_origin = g.scan_origin;

    std::vector<patch_t> patches(g.num_patches);
    for (uint32_t i = 0; i < g.num_patches; ++i) {
        patches[i].origin = g.origins[i];
        patches[i].local_bbox = g.bboxes[i];
        patches[i].base = g.bases[i];

        chunk_ptr_t chunk_jbig2(new chunk_t(cloud->patch_image_data[i*2 + 0]));
        chunk_ptr_t chunk_jpeg2k(new chunk_t(cloud->patch_image_data[i*2 + 1]));

        patches[i].occ_map = jbig2_decompress_chunk(chunk_jbig2);
        patches[i].height_map = jpeg2000_decompress_chunk(chunk_jpeg2k);
    }

    return patches;
}

} // pcl_compress
