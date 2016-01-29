#include <iostream>
#include <iomanip>
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

#include <e57_pcl/read.hpp>

using namespace pcl_compress;

int
main(int argc, char const* argv[]) {
    std::string file_in;
    std::string folder_out;
    vec2i_t img_size;
    uint32_t blur_iters;
    int32_t max_points;
    uint32_t quality;

    po::options_description desc("jpeg2000_test command line options");
    desc.add_options()("help,h", "Help message")
        ("input-file,i", po::value<std::string>(&file_in)->required(), "Input file")
        ("output-folder,o", po::value<std::string>(&folder_out)->required(), "Output folder")
        ("img-size,s", po::value<int>(&img_size[0])->default_value(32), "Image width and height")
        ("blur-iterations,b", po::value<uint32_t>(&blur_iters)->default_value(8), "Number of blur iterations")
        ("max-points-per-cell,m", po::value<int32_t>(&max_points)->default_value(-1), "Point count threshold for subdividing quadtree cells (Default: -1 => Use img-size * img-size).")
        ("quality,q", po::value<uint32_t>(&quality)->default_value(35), "JPEG2000 quality setting (try 35-40)")
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
    fs::path path_out(folder_out);

    if (!fs::exists(file_in)) {
        std::cerr << "Input file does not exist"
                  << "\n";
        return 1;
    }

    if (!fs::exists(folder_out)) {
        fs::create_directories(folder_out);
    }

    prim_detect_params_t params = {
        20000,    // min_points
        0.05f,  // angle_threshold
        0.05f,  // epsilon
        0.1f,   // bitmap_epsilon
        0.f,    // min_area
        0.001f  // probability_threshold
    };

    uint32_t scan_count = e57_pcl::get_scan_count(path_in.string());
    //Eigen::Vector3d first_scan_origin = e57_pcl::get_first_scan_origin(path_in.string());
    for (uint32_t scan_idx = 0; scan_idx < scan_count; ++scan_idx) {
        std::string guid;

        std::cout << "processing scan " << scan_idx << "..." << "\n";
        cloud_normal_t::Ptr cloud_in = e57_pcl::load_e57_scans_with_normals(
            path_in.string(), guid, true, nullptr, {scan_idx})[0];

        bbox3f_t bbox;
        for (const auto& p : cloud_in->points) {
            bbox.extend(p.getVector3fMap());
        }

        std::cout << "\tcomputing patches..." << "\n";
        decomposition_t decomp = primitive_decomposition<point_normal_t>(
            cloud_in, params, max_points, 6, 0.2f/*0.03f * bbox.diagonal().norm()*/);

        std::vector<patch_t> patches;
        for (const auto& subset : decomp) {
            patch_t patch =
                compute_patch(cloud_in, subset, img_size, blur_iters);
            patches.push_back(patch);
        }

        std::cout << "\tcompressing..." << "\n";
        vec3f_t scan_origin = cloud_in->sensor_origin_.head(3);
        compressed_cloud_t::ptr_t cc = compress_patches(patches, quality, scan_idx, scan_origin);

        std::stringstream out_name;
        out_name << (path_out / fs::basename(file_in)).string() << "_" << std::setfill('0') << std::setw(static_cast<int>(std::floor(std::log10(scan_count-1)))+1) << scan_idx << ".e57c";
        serialize(BINARY, out_name.str(), *cc);
    }
}
