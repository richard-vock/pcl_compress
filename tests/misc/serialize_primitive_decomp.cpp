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

    po::options_description desc("jpeg2000_test command line options");
    desc.add_options()("help,h", "Help message")(
        "input-file,i", po::value<std::string>(&file_in)->required(),
        "Input file")("output-file,o",
                      po::value<std::string>(&file_out)->required(),
                      "Output file");

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
        200, // min_points
        0.05f, // angle_threshold
        0.01f, // epsilon
        0.5f,  // bitmap_epsilon
        0.f,   // min_area
        0.001f // probability_threshold
    };
    decomposition_t decomp = primitive_decomposition<point_normal_t>(
        cloud_in,
        params,
        1024,
        6,
        0.03f * bbox.diagonal().norm());

    serialize(BINARY, file_out, decomp);
}
