#include <iostream>
#include <fstream>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <sys/fcntl.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
namespace fs = boost::filesystem;
namespace po = boost::program_options;

#include <compress.hpp>
#include <decompress.hpp>
#include <decomposition.hpp>

#include <pcl/io/pcd_io.h>

using namespace pcl_compress;

int
main(int argc, char const* argv[]) {
    std::string file_in;
    std::string file_out;
    vec2i_t img_size;
    uint32_t blur_iters;
    int32_t max_points;

    po::options_description desc("jpeg2000_test command line options");
    desc.add_options()("help,h", "Help message")(
        "input-file,i", po::value<std::string>(&file_in)->required(), "Input file")
        ("output-file,o", po::value<std::string>(&file_out)->required(), "Output file")
        ("img-size,s", po::value<int>(&img_size[0])->default_value(32), "Image width and height")
        ("blur-iterations,b", po::value<uint32_t>(&blur_iters)->default_value(8), "Number of blur iterations")
        ("max-points-per-cell,m", po::value<int32_t>(&max_points)->default_value(-1), "Point count threshold for subdividing quadtree cells (Default: -1 => Use img-size * img-size).")
    ;

    // Check for required options.
    po::variables_map vm;
    bool optionsException = false;
    try {
        po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
        // po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);
    } catch (std::exception& e) {
        if (!vm.count("help")) {
            std::cout << e.what() << "\n";
        }
        optionsException = true;
    }
    if (optionsException || vm.count("help")) {
        std::cout << desc << "\n";
        return optionsException ? 1 : 0;
    }

    img_size[1] = img_size[0];
    if (max_points < 0) max_points = img_size[0] * img_size[1];

    fs::path path_in(file_in);
    fs::path path_out(file_out);

    if (!fs::exists(file_in)) {
        std::cerr << "Input file does not exist"
                  << "\n";
        return 1;
    }

    cloud_normal_t::Ptr cloud_in(new cloud_normal_t());
    pcl::io::loadPCDFile(path_in.string(), *cloud_in);

    bbox3f_t bbox;
    for (const auto& p : cloud_in->points) {
        bbox.extend(p.getVector3fMap());
    }
    prim_detect_params_t params = {
        200,    // min_points
        0.05f,  // angle_threshold
        0.01f,  // epsilon
        0.5f,   // bitmap_epsilon
        0.f,    // min_area
        0.001f  // probability_threshold
    };
    decomposition_t decomp = primitive_decomposition<point_normal_t>(
        cloud_in, params, max_points, 6, 0.03f * bbox.diagonal().norm());

    std::vector<patch_t> patches;
    for (const auto& subset : decomp) {
        patch_t patch = compute_patch(cloud_in, subset, img_size, blur_iters);
        patches.push_back(patch);
    }

    std::cout << "compressing"
              << "\n";
    compressed_cloud_t::ptr_t cc = compress_patches(patches, 35);
    uint32_t chars = 4 * 4 + 2 * cc->origins.size() + 2 * cc->bboxes.size() +
                     cc->bases.size();
    for (const auto& chunk : cc->patch_image_data) {
        chars += chunk.size();
    }

    uint64_t bytes_in = static_cast<uint64_t>(cloud_in->size() * 12);
    uint64_t bytes_out = static_cast<uint64_t>(chars);
    uint64_t c_factor = bytes_in / bytes_out;
    double bpp = static_cast<double>(bytes_out) /
                 (static_cast<double>(cc->num_points) / 8.0);
    std::cout << "compression factor: " << c_factor << "\n";
    std::cout << "bpp: " << bpp << "\n";

    serialize(BINARY, file_out, *cc);

    //compressed_cloud_t::ptr_t cc_test(new compressed_cloud_t());
    //deserialize(BINARY, file_out, *cc_test)
}
