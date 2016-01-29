#include <iostream>
#include <fstream>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
namespace fs = boost::filesystem;
namespace po = boost::program_options;

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <e57_pcl/read.hpp>
using namespace e57_pcl;




int main (int argc, char const* argv[]) {
    std::string  original_file, compressed_file;
    uint32_t index;

	po::options_description desc("decompress_patches command line options");
	desc.add_options()
		("help,h",  "Help message")
		("original,o",     po::value<std::string>(&original_file), "Original File")
		("compressed,c",    po::value<std::string>(&compressed_file)->required(), "Compressed File")
		("index",    po::value<uint32_t>(&index)->required(), "Scan Index")
	;

    //po::positional_options_description p;
    //p.add("input-file", -1);

	// Check for required options.
	po::variables_map vm;
	bool optionsException = false;
	try {
		po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
		//po::store(po::parse_command_line(argc, argv, desc), vm);
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

    //std::cout << "Loading original file (scan: " << index << ")" << "\n";
    std::string guid;
    cloud_normal_t::Ptr orig = load_e57_scans_with_normals(original_file, guid, true, nullptr, {index})[0];
    //pcl::io::savePCDFileBinary("/tmp/original_scan.pcd", *orig);

    //std::cout << "Loading compressed file..." << "\n";
    cloud_xyz_t::Ptr compr(new cloud_xyz_t());
    pcl::io::loadPCDFile(compressed_file, *compr);

    pcl::KdTreeFLANN<point_normal_t> kdtree;
    kdtree.setInputCloud(orig);

    std::vector<int> results;
    std::vector<float> sqr_dists;
    uint32_t n = 0;
    float mean = 0.f, delta = 0.f;
    for (const auto& p : *compr) {
        kdtree.template nearestKSearchT<point_xyz_t>(p, 1, results, sqr_dists);
        // knuth
        ++n;
        delta = sqr_dists[0] - mean;
        mean += delta / static_cast<float>(n);
    }

    std::cout << "\t" << compr->size() << "\t" << sqrt(mean) << "\n";
    //std::cout << "\tRMSE: " << sqrtf(mean) << "\n";
}
