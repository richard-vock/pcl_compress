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

#include <e57_pcl/read.hpp>
using namespace e57_pcl;


int
main(int argc, char const* argv[]) {
    std::string file_in;

    po::options_description desc("jpeg2000_test command line options");
    desc.add_options()("help,h", "Help message")
        ("input-file,i", po::value<std::string>(&file_in)->required(), "Input file")
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

    fs::path path_in(file_in);

    if (!fs::exists(file_in)) {
        std::cerr << "Input file does not exist"
                  << "\n";
        return 1;
    }


    uint32_t scan_count = e57_pcl::get_scan_count(path_in.string());

    std::cout << "scan count: " << scan_count << "\n";
    uint64_t point_count = 0;
    //Eigen::Vector3d first_scan_origin = e57_pcl::get_first_scan_origin(path_in.string());
    for (uint32_t scan_idx = 0; scan_idx < scan_count; ++scan_idx) {
        std::string guid;
        cloud_normal_t::Ptr cloud_in = e57_pcl::load_e57_scans_with_normals(
            path_in.string(), guid, true, nullptr, {scan_idx})[0];
        point_count += static_cast<uint64_t>(cloud_in->size());
    }

    std::cout << "point count: " << point_count << "\n";
    std::cout << "average per scan: " << (point_count / static_cast<uint64_t>(scan_count)) << "\n";
    std::cout << "size in byte: " << (point_count * 6l * 4l) << "\n";

}
