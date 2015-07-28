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

#include <metrics.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree.h>

using namespace pcl_compress;


int main (int argc, char const* argv[]) {
	std::string  file_in;
	std::string  file_out;

	po::options_description desc("jpeg2000_test command line options");
	desc.add_options()
		("help,h",  "Help message")
		("input-file,i",     po::value<std::string>(&file_in)->required(), "Input file")
		("output-file,o",    po::value<std::string>(&file_out)->required(), "Output file")
	;

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

    fs::path path_in(file_in);
    fs::path path_out(file_out);

    if (!fs::exists(file_in)) {
        std::cerr << "Input file does not exist" << "\n";
        return 1;
    }

    cloud_xyz_t::Ptr cloud_in(new cloud_xyz_t());
    pcl::io::loadPCDFile(path_in.string(), *cloud_in);
    std::cout << cloud_in->size() << "\n";

    bbox3f_t bbox;
    for (const auto& p : cloud_in->points) {
        bbox.extend(p.getVector3fMap());
    }
    pcl::octree::OctreePointCloud<point_xyz_t> octree(0.03f * bbox.diagonal().norm());
    octree.setInputCloud(cloud_in);
    octree.addPointsFromInputCloud();

    std::vector<patch_t> patches;
    for (auto it = octree.leaf_begin(); it != octree.leaf_end(); ++it) {
        std::vector<int> subset;
        it.getLeafContainer().getPointIndices(subset);
        if (subset.size() < 5) continue;
        patch_t patch = compute_patch(cloud_in, subset, 100.f, 1E-3);
        patches.push_back(patch);
    }

    uint32_t idx = 0;
    for (const auto& patch : patches) {
        std::string img_fn = "/tmp/original_"+std::to_string(idx++)+".png";
        cv::imwrite(img_fn, patch.occ_map);
    }


    std::cout << "compressing" << "\n";
    compressed_cloud_t::ptr_t cc = compress_patches(patches, 35);
    uint32_t chars = 4 * 4 + 2*cc->origins.size() + 2*cc->bboxes.size() + cc->bases.size();
    for (const auto& chunk : cc->patch_image_data) {
        chars += chunk.size();
    }

    uint64_t bytes_in = static_cast<uint64_t>(cloud_in->size() * 12);
    uint64_t bytes_out = static_cast<uint64_t>(chars);
    uint64_t c_factor = bytes_in / bytes_out;
    double bpp = static_cast<double>(bytes_out) / (static_cast<double>(cc->num_points) / 8.0);
    std::cout << "compression factor: " << c_factor << "\n";
    std::cout << "bpp: " << bpp << "\n";
    std::cout << "decompressing" << "\n";
    std::vector<patch_t> dec_patches = decompress_patches(cc);
    std::cout << "done" << "\n";

    cloud_xyz_t::Ptr cloud_out = from_patches(dec_patches);
    pcl::io::savePCDFileBinary(path_out.string(), *cloud_out);
}
